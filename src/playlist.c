/* playlist.c — playlist mode: play pre-encoded AMBE files on a timer.
 *
 * An instance with a playlist_dir is a player rather than a talkback.  Every
 * playlist_interval seconds each of its lanes plays the next .amb file from the
 * directory (and its immediate subdirectories) as one group call on its
 * talkgroup, sourced from the instance's radio ID.  Inbound calls are ignored
 * (capture.c).  Made for repeatable audio-quality / network tests: the same AMBE
 * goes out bit-identical every time, so any difference a listener hears is the
 * path, not the source.
 *
 * With playlist_pick > 1 each transmission is instead that many files picked at
 * random from ONE randomly chosen subdirectory (e.g. one speaker), joined with
 * playlist_gap_ms of AMBE silence between them, in a single call.
 *
 * The .amb files come from md380emu's encoder (an 8-byte record per 20 ms frame:
 * a status byte, then the 49 AMBE bits MSB-first with the last in byte 7's LSB).
 * This file only BUILDS the call -- header, bursts A..F with the right sync and
 * EMB fields, terminator -- into a lane's capture buffer; replay.c then rewrites
 * the addressing and LC and clocks it out exactly as it does for an echo.
 */
#include "talkback.h"
#include "hbp.h"
#include "hbp_const.h"
#include "log.h"
#include "dmr/dmr.h"
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define LOGN "talkback.playlist"

#define AMB_RECORD   8
#define AMB_MAGIC    ".amb"
#define FRAMES_PER_BURST 3

/* 72-bit AMBE silence (dmr_utils3; the same filler ipsc2hbp uses), for padding
 * the last burst of a call to three frames. */
static const uint8_t AMBE_SILENCE_72[9] = { 0xAC, 0xAA, 0x40, 0x20, 0x00, 0x44, 0x40, 0x80, 0x80 };

/* ---------------- pure builders (exercised directly by tests) ---------------- */

static void packet_header(uint8_t *pkt, uint8_t flags) {
    memset(pkt, 0, DMRD_LEN);
    memcpy(pkt, "DMRD", 4);
    pkt[DMRD_FLAGS_OFF] = flags;
}

/* Voice LC header / terminator: [98:108] slot type, [108:156] data sync,
 * [156:166] slot type.  The LC windows stay zero -- tb_rewrite_packet fills them. */
static void build_lc_frame(uint8_t *pkt, uint8_t flags, const dmr_bit slot_type[20]) {
    packet_header(pkt, flags);
    dmr_bit bits[264];
    memset(bits, 0, sizeof bits);
    memcpy(bits + 98,  slot_type,        10);
    memcpy(bits + 108, DMR_BS_DATA_SYNC, 48);
    memcpy(bits + 156, slot_type + 10,   10);
    dmr_bits_to_bytes(bits, 264, pkt + DMRD_PAYLOAD_OFF);
}

static void frame_72(const uint8_t (*ambe49)[7], int nframes, int idx, dmr_bit out72[72]) {
    if (idx >= nframes) {                       /* past the end: silence */
        dmr_bytes_to_bits(AMBE_SILENCE_72, 9, out72);
        return;
    }
    dmr_bit b49[56];
    dmr_bytes_to_bits(ambe49[idx], 7, b49);     /* 49 bits used, MSB-first */
    dmr_ambe_49_to_72(b49, out72);
}

int tb_build_call(const uint8_t (*ambe49)[7], int nframes, int slot,
                  uint8_t *pkts, int cap)
{
    int nbursts = (nframes + FRAMES_PER_BURST - 1) / FRAMES_PER_BURST;
    int total = nbursts + 2;                    /* + header + terminator */
    if (nframes <= 0 || total > cap) return -1;
    uint8_t ts = (slot == 2) ? HBPF_TGID_TS2 : 0;

    uint8_t *p = pkts;
    build_lc_frame(p, ts | HBPF_FRAMETYPE_DATASYNC | HBPF_SLT_VHEAD, DMR_SLOT_TYPE_VHEAD);
    p += DMRD_LEN;

    for (int b = 0; b < nbursts; b++, p += DMRD_LEN) {
        int pos = b % 6;                        /* superframe position, A..F */
        dmr_bit a1[72], a2[72], a3[72], bits[264];
        frame_72(ambe49, nframes, b * 3,     a1);
        frame_72(ambe49, nframes, b * 3 + 1, a2);
        frame_72(ambe49, nframes, b * 3 + 2, a3);

        /* Middle 48 bits: voice sync on A; on B..F the 16-bit EMB header split
         * around a 32-bit embedded-LC fragment (left zero: rewritten for B..E,
         * the null fragment on F). */
        dmr_bit mid[48];
        if (pos == 0) {
            memcpy(mid, DMR_BS_VOICE_SYNC, 48);
        } else {
            memset(mid, 0, sizeof mid);
            memcpy(mid,      DMR_EMB[pos - 1],     8);
            memcpy(mid + 40, DMR_EMB[pos - 1] + 8, 8);
        }
        memcpy(bits,       a1,      72);
        memcpy(bits + 72,  a2,      36);
        memcpy(bits + 108, mid,     48);
        memcpy(bits + 156, a2 + 36, 36);
        memcpy(bits + 192, a3,      72);

        packet_header(p, ts | (pos == 0 ? HBPF_FRAMETYPE_VOICESYNC
                                        : (uint8_t)(HBPF_FRAMETYPE_VOICE | pos)));
        dmr_bits_to_bytes(bits, 264, p + DMRD_PAYLOAD_OFF);
    }

    build_lc_frame(p, ts | HBPF_FRAMETYPE_DATASYNC | HBPF_SLT_VTERM, DMR_SLOT_TYPE_VTERM);
    return total;
}

int tb_read_amb(const char *path, uint8_t (*out)[7], int max_frames)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    char magic[4];
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, AMB_MAGIC, 4) != 0) { fclose(f); return -1; }
    int n = 0;
    uint8_t rec[AMB_RECORD];
    while (n < max_frames && fread(rec, 1, AMB_RECORD, f) == AMB_RECORD) {
        memcpy(out[n], rec + 1, 6);             /* bits 0..47 */
        out[n][6] = (uint8_t)((rec[7] & 1) << 7);  /* bit 48 */
        n++;
    }
    fclose(f);
    return n;
}

/* ---------------- the player ---------------- */

typedef struct {
    tb_instance *inst;
    char       **files;
    int          nfiles;
    int         *dir_of;        /* file -> index into dirs */
    int          ndirs;
    int         *dir_first;     /* files are sorted, so each dir is a contiguous run */
    int         *dir_count;
    uint8_t      silence49[7];  /* one frame of AMBE silence, packed 49-bit */
    int         *order;         /* play order, a permutation of 0..nfiles-1 */
    int          next;
    uint8_t    (*frames)[7];    /* scratch, sized from max_capture_secs */
    int          max_frames;
} playlist;

static int cmp_str(const void *a, const void *b) {
    return strcmp(*(char *const *)a, *(char *const *)b);
}

static int has_amb_suffix(const char *name) {
    size_t n = strlen(name);
    return n > 4 && !strcmp(name + n - 4, ".amb");
}

static void add_file(playlist *pl, const char *path) {
    char **f = realloc(pl->files, (size_t)(pl->nfiles + 1) * sizeof *f);
    if (!f) return;
    pl->files = f;
    pl->files[pl->nfiles] = strdup(path);
    if (pl->files[pl->nfiles]) pl->nfiles++;
}

/* The directory's .amb files and those one level down (e.g. one folder per
 * speaker set). */
static void scan(playlist *pl, const char *dir, int depth) {
    DIR *d = opendir(dir);
    if (!d) return;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        char path[1024];
        snprintf(path, sizeof path, "%s/%s", dir, de->d_name);
        struct stat st;
        if (stat(path, &st) != 0) continue;
        if (S_ISDIR(st.st_mode) && depth == 0) scan(pl, path, 1);
        else if (S_ISREG(st.st_mode) && has_amb_suffix(de->d_name)) add_file(pl, path);
    }
    closedir(d);
}

static void shuffle(playlist *pl) {
    for (int i = pl->nfiles - 1; i > 0; i--) {
        int j = rand() % (i + 1), t = pl->order[i];
        pl->order[i] = pl->order[j];
        pl->order[j] = t;
    }
}

static void tick_cb(ev_loop *loop, void *ud);

/* Length of a path's directory part ("a/b/c.amb" -> 3), 0 if none. */
static size_t dir_len(const char *path) {
    const char *slash = strrchr(path, '/');
    return slash ? (size_t)(slash - path) : 0;
}

/* Group the (sorted) files by parent directory: each directory is a contiguous run. */
static int index_dirs(playlist *pl) {
    pl->dir_of = malloc((size_t)pl->nfiles * sizeof *pl->dir_of);
    pl->dir_first = malloc((size_t)pl->nfiles * sizeof *pl->dir_first);
    pl->dir_count = calloc((size_t)pl->nfiles, sizeof *pl->dir_count);
    if (!pl->dir_of || !pl->dir_first || !pl->dir_count) return -1;
    for (int i = 0; i < pl->nfiles; i++) {
        size_t len = dir_len(pl->files[i]);
        int new_dir = i == 0 || len != dir_len(pl->files[i - 1]) ||
                      strncmp(pl->files[i], pl->files[i - 1], len) != 0;
        if (new_dir) pl->dir_first[pl->ndirs++] = i;
        pl->dir_of[i] = pl->ndirs - 1;
        pl->dir_count[pl->ndirs - 1]++;
    }
    return 0;
}

/* Append a file's frames at `at`; returns the new frame count, or -1 if unreadable
 * or it would overflow the scratch buffer. */
static int append_file(playlist *pl, const char *path, int at) {
    int n = tb_read_amb(path, pl->frames + at, pl->max_frames - at);
    if (n <= 0) return -1;
    return at + n;
}

/* One transmission of playlist_pick random files from one random directory. */
static void play_group(playlist *pl) {
    tb_instance *in = pl->inst;
    const InstanceCfg *ic = instance_cfg(in);
    int d = rand() % pl->ndirs;
    int first = pl->dir_first[d], count = pl->dir_count[d];
    int pick = ic->playlist_pick < count ? ic->playlist_pick : count;

    int chosen[20], nframes = 0;
    char names[512] = "";
    for (int k = 0; k < pick; k++) {
        int f, dup;
        do {                                    /* distinct files within the dir */
            f = first + rand() % count;
            dup = 0;
            for (int j = 0; j < k; j++) if (chosen[j] == f) dup = 1;
        } while (dup);
        chosen[k] = f;
        if (k > 0) {                            /* the gap between files */
            for (int g = 0; g < ic->playlist_gap_frames && nframes < pl->max_frames; g++)
                memcpy(pl->frames[nframes++], pl->silence49, 7);
        }
        int n = append_file(pl, pl->files[f], nframes);
        if (n < 0) {
            LOGW(LOGN, "[%s] %s: unreadable or too long for max_capture_secs — transmission skipped",
                 ic->name, pl->files[f]);
            return;
        }
        nframes = n;
        const char *base = strrchr(pl->files[f], '/');
        base = base ? base + 1 : pl->files[f];
        size_t used = strlen(names);
        snprintf(names + used, sizeof names - used, "%s%s", k ? ", " : "", base);
    }

    for (int slot = 1; slot <= 2; slot++) {
        tb_lane *ln = instance_lane(in, slot);
        if (!ln || !ln->tgid) continue;
        if (lane_replay_active(ln)) {
            LOGW(LOGN, "[%s] TS%d still playing — group skipped on this slot", ic->name, slot);
            continue;
        }
        int n = tb_build_call((const uint8_t (*)[7])pl->frames, nframes, slot, ln->cap.pkts, ln->cap.cap);
        if (n < 0) {
            LOGW(LOGN, "[%s] %.1f s group is longer than max_capture_secs — skipped",
                 ic->name, nframes * 0.02);
            continue;
        }
        ln->cap.n = n;
        LOGI(LOGN, "[%s] TS%d playing %s (%.1f s) on TG %u", ic->name, slot, names,
             nframes * 0.02, ln->tgid);
        lane_replay_start(ln);
    }
}

static void play_next(playlist *pl) {
    tb_instance *in = pl->inst;
    const InstanceCfg *ic = instance_cfg(in);
    struct hbp *hb = instance_hbp(in);
    if (!hb || !hbp_is_connected(hb)) {
        LOGD(LOGN, "[%s] not connected — skipping this interval", ic->name);
        return;
    }
    if (ic->playlist_pick > 1) { play_group(pl); return; }

    if (pl->next >= pl->nfiles) {               /* end of a cycle */
        pl->next = 0;
        if (ic->playlist_shuffle) shuffle(pl);
    }
    const char *path = pl->files[pl->order[pl->next++]];
    int nframes = tb_read_amb(path, pl->frames, pl->max_frames);
    if (nframes <= 0) {
        LOGW(LOGN, "[%s] %s: not a readable .amb file — skipped", ic->name, path);
        return;
    }
    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;

    for (int slot = 1; slot <= 2; slot++) {
        tb_lane *ln = instance_lane(in, slot);
        if (!ln || !ln->tgid) continue;
        if (lane_replay_active(ln)) {
            LOGW(LOGN, "[%s] TS%d still playing — %s skipped on this slot", ic->name, slot, base);
            continue;
        }
        int n = tb_build_call((const uint8_t (*)[7])pl->frames, nframes, slot, ln->cap.pkts, ln->cap.cap);
        if (n < 0) {
            LOGW(LOGN, "[%s] %s: %.1f s is longer than max_capture_secs — skipped",
                 ic->name, base, nframes * 0.02);
            continue;
        }
        ln->cap.n = n;
        LOGI(LOGN, "[%s] TS%d playing %s (%d/%d, %.1f s) on TG %u",
             ic->name, slot, base, pl->next, pl->nfiles, nframes * 0.02, ln->tgid);
        lane_replay_start(ln);
    }
}

static void tick_cb(ev_loop *loop, void *ud) {
    playlist *pl = ud;
    /* Re-arm first, so the interval is start to start. */
    ev_timer_after(loop, instance_cfg(pl->inst)->playlist_interval, tick_cb, pl);
    play_next(pl);
}

int instance_playlist_start(tb_instance *in)
{
    const InstanceCfg *ic = instance_cfg(in);
    playlist *pl = calloc(1, sizeof *pl);   /* lives as long as the process */
    if (!pl) return -1;
    pl->inst = in;
    scan(pl, ic->playlist_dir, 0);
    if (pl->nfiles == 0) { free(pl); return -1; }
    qsort(pl->files, (size_t)pl->nfiles, sizeof *pl->files, cmp_str);

    pl->order = malloc((size_t)pl->nfiles * sizeof *pl->order);
    pl->max_frames = ic->max_capture_secs * 50;              /* 20 ms frames */
    pl->frames = malloc((size_t)pl->max_frames * sizeof *pl->frames);
    if (!pl->order || !pl->frames) return -1;
    for (int i = 0; i < pl->nfiles; i++) pl->order[i] = i;
    if (ic->playlist_shuffle) shuffle(pl);
    if (index_dirs(pl) != 0) return -1;
    {   dmr_bit s72[72], s49[56];
        dmr_bytes_to_bits(AMBE_SILENCE_72, 9, s72);
        memset(s49, 0, sizeof s49);
        dmr_ambe_72_to_49(s72, s49);
        dmr_bits_to_bytes(s49, 56, pl->silence49);
    }

    if (ic->playlist_pick > 1)
        LOGI(LOGN, "[%s] playlist: %d files in %d directories from %s; every %.0f s, %d random "
                   "files from one random directory, %d ms apart",
             ic->name, pl->nfiles, pl->ndirs, ic->playlist_dir, ic->playlist_interval,
             ic->playlist_pick, ic->playlist_gap_frames * 20);
    else
        LOGI(LOGN, "[%s] playlist: %d files from %s, %s, one every %.0f s",
             ic->name, pl->nfiles, ic->playlist_dir,
             ic->playlist_shuffle ? "shuffled" : "in order", ic->playlist_interval);
    ev_timer_after(instance_loop(in), ic->playlist_interval, tick_cb, pl);
    return 0;
}
