/* test_playlist.c — building a group call from pre-encoded AMBE (playlist mode).
 *
 * The player's whole job is to turn a list of 49-bit AMBE frames into DMRD
 * packets that replay.c can clock out and a repeater will play.  What must hold:
 *
 *   1. Frame kinds: voice header, bursts A..F repeating (A = voice sync, B..F =
 *      voice with vseq 1..5), terminator; the slot bit on every packet.
 *   2. Sync and slot type sit where a receiver looks for them.
 *   3. The AMBE recovered from the packets is bit-identical to the input, with
 *      AMBE silence padding the last burst.
 *   4. After the normal replay rewrite, the header's LC decodes to the right
 *      talkgroup and source -- i.e. the built frame and the rewrite agree.
 *   5. tb_read_amb reads md380emu's .amb layout.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "talkback.h"
#include "hbp.h"
#include "hbp_const.h"
#include "dmr/dmr.h"

/* The builder never reaches the wire. */
void hbp_send_dmrd(struct hbp *hb, const uint8_t *data, int len) { (void)hb; (void)data; (void)len; }
int  hbp_is_connected(struct hbp *hb) { (void)hb; return 0; }

static int failures = 0;
static int checks   = 0;
#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static const uint8_t SILENCE_72[9] = { 0xAC, 0xAA, 0x40, 0x20, 0x00, 0x44, 0x40, 0x80, 0x80 };

static void payload_bits(const uint8_t *pkt, dmr_bit bits[264]) {
    dmr_bytes_to_bits(pkt + DMRD_PAYLOAD_OFF, 33, bits);
}

/* The three 72-bit AMBE frames of a voice burst, back to 49 bits each. */
static void burst_frames(const uint8_t *pkt, dmr_bit out[3][49]) {
    dmr_bit b[264], a2[72];
    payload_bits(pkt, b);
    memcpy(a2, b + 72, 36);
    memcpy(a2 + 36, b + 156, 36);
    dmr_ambe_72_to_49(b, out[0]);
    dmr_ambe_72_to_49(a2, out[1]);
    dmr_ambe_72_to_49(b + 192, out[2]);
}

static void test_build(int slot) {
    enum { NF = 20 };                     /* 20 frames = 6 full bursts + 2 frames */
    uint8_t in[NF][7];
    srand(12345 + slot);
    for (int i = 0; i < NF; i++) {
        for (int k = 0; k < 7; k++) in[i][k] = (uint8_t)rand();
        in[i][6] &= 0x80;                 /* only bit 48 used in the last byte */
    }
    uint8_t pkts[16 * DMRD_LEN];
    int n = tb_build_call((const uint8_t (*)[7])in, NF, slot, pkts, 16);
    CHECK(n == 3 + 7 + 1, "slot %d: %d packets, want 11 (3 headers + 7 bursts + terminator)", slot, n);
    if (n != 11) return;

    uint8_t ts = slot == 2 ? HBPF_TGID_TS2 : 0;
    for (int i = 0; i < n; i++) {
        const uint8_t *p = pkts + i * DMRD_LEN;
        uint8_t f = p[DMRD_FLAGS_OFF];
        CHECK(!memcmp(p, "DMRD", 4), "pkt %d: DMRD magic", i);
        CHECK((f & HBPF_TGID_TS2) == ts, "pkt %d: slot bit", i);
        uint8_t kind = f & (HBPF_FRAMETYPE_MASK | HBPF_DTYPE_MASK);
        if (i < 3)           CHECK(kind == (HBPF_FRAMETYPE_DATASYNC | HBPF_SLT_VHEAD), "pkt %d is a voice header", i);
        else if (i == n - 1) CHECK(kind == (HBPF_FRAMETYPE_DATASYNC | HBPF_SLT_VTERM), "last pkt is a terminator");
        else {
            int pos = (i - 3) % 6;
            uint8_t want = pos == 0 ? HBPF_FRAMETYPE_VOICESYNC : (uint8_t)(HBPF_FRAMETYPE_VOICE | pos);
            CHECK(kind == want, "pkt %d: burst %c flags %02x, want %02x", i, 'A' + pos, kind, want);
        }
    }

    /* Sync and slot type */
    dmr_bit b[264];
    payload_bits(pkts, b);
    CHECK(!memcmp(b + 108, DMR_BS_DATA_SYNC, 48), "header carries data sync");
    CHECK(!memcmp(b + 98, DMR_SLOT_TYPE_VHEAD, 10) && !memcmp(b + 156, DMR_SLOT_TYPE_VHEAD + 10, 10),
          "header slot type");
    payload_bits(pkts + 3 * DMRD_LEN, b);
    CHECK(!memcmp(b + 108, DMR_BS_VOICE_SYNC, 48), "burst A carries voice sync");
    payload_bits(pkts + 4 * DMRD_LEN, b);
    CHECK(!memcmp(b + 108, DMR_EMB[0], 8) && !memcmp(b + 148, DMR_EMB[0] + 8, 8), "burst B EMB header");
    payload_bits(pkts + 10 * DMRD_LEN, b);
    CHECK(!memcmp(b + 108, DMR_BS_DATA_SYNC, 48), "terminator carries data sync");
    CHECK(!memcmp(b + 98, DMR_SLOT_TYPE_VTERM, 10), "terminator slot type");

    /* AMBE round trip: 7 bursts x 3 = 21 frames, the 21st silence */
    int bad = 0;
    for (int bi = 0; bi < 7; bi++) {
        dmr_bit got[3][49];
        burst_frames(pkts + (3 + bi) * DMRD_LEN, got);
        for (int j = 0; j < 3; j++) {
            int idx = bi * 3 + j;
            dmr_bit want[56], s72[72];
            if (idx < NF) dmr_bytes_to_bits(in[idx], 7, want);
            else { dmr_bytes_to_bits(SILENCE_72, 9, s72); dmr_ambe_72_to_49(s72, want); }
            if (memcmp(got[j], want, 49)) bad++;
        }
    }
    CHECK(bad == 0, "slot %d: %d of 21 AMBE frames differ after the build", slot, bad);

    /* Rewrite the header as replay.c would; its LC must decode to our addressing */
    tb_rewrite rw;
    uint8_t dst[3] = { 0x00, 0x27, 0x0D };              /* TG 9997 */
    tb_rewrite_init(&rw, 13027161u, dst, 0xABCD1234u);
    uint8_t out[DMRD_LEN], lc[9];
    tb_rewrite_packet(&rw, pkts, out, 0);
    payload_bits(out, b);
    dmr_bit full[196];
    memcpy(full, b, 98);
    memcpy(full + 98, b + 166, 98);
    dmr_bptc_decode_full_lc(full, lc);
    CHECK(lc[3] == 0x00 && lc[4] == 0x27 && lc[5] == 0x0D, "rewritten header LC: TG 9997");
    CHECK(((uint32_t)lc[6] << 16 | (uint32_t)lc[7] << 8 | lc[8]) == 13027161u, "rewritten header LC: source");
    CHECK(!memcmp(b + 108, DMR_BS_DATA_SYNC, 48), "rewrite leaves the header's sync alone");

    /* Too small a buffer is refused, not overrun */
    CHECK(tb_build_call((const uint8_t (*)[7])in, NF, slot, pkts, 10) == -1, "cap 10 < 11 refused");
}

static void test_read_amb(void) {
    const char *path = "/tmp/talkback_test_playlist.amb";
    FILE *f = fopen(path, "wb");
    fwrite(".amb", 1, 4, f);
    uint8_t rec[2][8] = { { 0, 0xF8, 0x21, 0xA0, 0x5E, 0x44, 0x79, 0x01 },
                          { 0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x00 } };
    fwrite(rec, 1, sizeof rec, f);
    fclose(f);
    uint8_t out[4][7];
    int n = tb_read_amb(path, out, 4);
    CHECK(n == 2, "read %d frames, want 2", n);
    CHECK(!memcmp(out[0], rec[0] + 1, 6) && out[0][6] == 0x80, "frame 0 bits (bit 48 set)");
    CHECK(!memcmp(out[1], rec[1] + 1, 6) && out[1][6] == 0x00, "frame 1 bits (bit 48 clear)");
    f = fopen(path, "wb"); fwrite("RIFF", 1, 4, f); fclose(f);
    CHECK(tb_read_amb(path, out, 4) == -1, "a file without the .amb magic is refused");
    remove(path);
}

int main(void) {
    test_build(1);
    test_build(2);
    test_read_amb();
    printf("playlist self-test: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
