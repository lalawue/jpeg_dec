// by suchang, 2014/11/22

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef signed char s8;
typedef signed short s16;
typedef signed int s32;

#define DCTSIZE 8
#define DCTSIZE2 64

#define VLC_MAX_LEN 16

#define M_SOI 0xffd8             // Start of image
#define M_EOI 0xffd9             // End of image
#define M_APP0 0xffe0            // Reserved for application segments
#define M_APPn 0xffef            
#define M_DQT 0xffdb             // Define quantization table
#define M_SOF0 0xffc0            // Baseline DCT
#define M_SOF2 0xffc2            // Baseline DCT
#define M_DHT 0xffc4             // Define Huffman table
#define M_SOS 0xffda             // Start of scan
#define M_DRI 0xffdd             // Define restart interval
#define M_COM 0xfffe             // Comment

enum { D_ERROR = 0, D_INFO, D_MARKER, D_COEFF, D_VERBOSE };

static int _debug_level = D_COEFF;

#define _log(LEV, ...)                          \
    do {                                        \
        if (_debug_level >= (LEV))              \
            printf(__VA_ARGS__);                \
    } while (0)

struct s_bctx {
    u8 *r_data;
    int len;
    int r_ptr;
    int rb_buf;                 /* bits buffer */
    int rb_bits;                /* bits for rb_buf holds */
    int r_eof;
};

int
_get_file_content(const char *filename, u8 **content, long *len) {
    FILE *fp = fopen(filename, "rb");
    if ( fp ) {
        fseek(fp, 0, SEEK_END);
        *len = ftell(fp);
        *content = malloc( *len );
        if ( content ) {
            memset(*content, 0, *len);
            rewind(fp);
            fread(*content, 1, *len, fp);
        }
        fclose(fp);
        return 1;
    }
    return 0;
}

void
_free_file_content(u8 *content) {
    if (content) free(content);
}

struct s_bctx*
_create_bctx( u8 *content, u32 len ) {
    struct s_bctx *b = malloc(sizeof(*b));
    if ( b ) {
        memset(b, 0, sizeof(*b));
        b->r_data = content;
        b->len = len;
    }
    return b;
}

void
_destroy_bctx(struct s_bctx *b) {
    if ( b ) {
        free(b);
    }
}

void
_dump_buf(const unsigned char *buf, int stride) {
    int i, base = 0;
    for (i=0; i<64; i++) {
        printf("%02x ", buf[base++]);
        if (i && (i+1)%8==0) {
            printf("\n");
            base += stride - 8;
        }
    }
}

void
_skip_bytes(struct s_bctx *b, int n) {
    b->r_ptr += n;
}

u8
_next_byte(struct s_bctx *b) {
    return b->r_data[b->r_ptr++];
}

u16
_next_word(struct s_bctx *b) {
    u8 b1 = _next_byte(b);
    return b1<<8 | _next_byte(b);
}

u8*
_get_ptr(struct s_bctx *b) {
    return &b->r_data[b->r_ptr];
}

u32
_get_offset(struct s_bctx *b) {
    return b->r_ptr;
}

int
_is_eof(struct s_bctx *b) {
    return (b->r_eof || b->r_ptr>=b->len);
}

void
_set_eof(struct s_bctx *b) {
    b->r_eof = 1;
}

u32
_bits_try(struct s_bctx *b, int n) {
    u8 buf;
    u32 data = 0;
    assert(n <= 32);
    while (b->rb_bits < n) {
        if ( _is_eof(b) ) {
            b->rb_buf = (b->rb_buf << 8) | 0xff;
            b->rb_bits += 8;
            continue;
        }
        buf = _next_byte(b);
        b->rb_buf <<= 8;
        b->rb_buf |= buf;
        b->rb_bits += 8;
        if (buf == 0xff) {
            u16 marker = _next_byte(b);
            switch( marker ) {
                case 0x00:
                case 0xff:
                    break;
                case (M_EOI & 0xff):
                    _skip_bytes(b, -2);
                    _set_eof(b);
                    break;
                default:
                    _log(D_ERROR, "# Should not reach here ! #\n");
                    assert(0);
            }
        }
    }
    data = b->rb_buf >> (b->rb_bits-n);
    return data;
}

void
_bits_skip(struct s_bctx *b, int n) {
    if (b->rb_bits >= n) {
        b->rb_bits -= n;        
        b->rb_buf &= ((1<<b->rb_bits) - 1);
    }
}

u32
_bits_read(struct s_bctx *b, int n) {
    u32 data = _bits_try(b, n);
    _bits_skip(b, n);
    return data;
}

void
_bits_clear(struct s_bctx *b) {
    b->rb_buf = 0;
    b->rb_bits = 0;
}

struct s_ht_vlc {
    u16 val;
    u8 code;
};

struct s_ht_ary {
    u8 count;                   /* vlc list count */
    struct s_ht_vlc *v;
};

struct s_ht_tbl {
    u8 count;                   /* vlc pairs count */
    struct s_ht_ary ary[VLC_MAX_LEN];    /* 0~15 bits */
};

struct s_jcomp {
    int id;
    int h_samp;                  /* horizontal sampling factor */
    int v_samp;                  /* vertical sampling factor */
    int h_mcus;                  /* horizontal MCU unit */
    int v_mcus;                  /* vertical MCU unit */
    int qtbl_id;
    int ht_dc_id;
    int ht_ac_id;
    int dc;
    int vec[DCTSIZE2];
    u8 pixels[DCTSIZE2];
};

struct s_jctx {
    int width;
    int height;
    u8 *qtbl[2];
    int htbl_count;
    int comp_count;
    struct s_ht_tbl htbl[4];
    struct s_jcomp comp[3];

    int mcu_sizex;              /* mcu width */
    int mcu_sizey;              /* mcu height */
    int h_mcus;                 /* horizontal MCU count */
    int v_mcus;                 /* vertical MCU count */
    int mcu_blocks;             /* max mcu blocks */

    int restintv;               /* rest interval */
    int restintv_next;          /* next */
    int restintv_cnt;           /* count */

    u8 *scan_out;               /* one line mcus */
    int scan_len;

    u8 *pixels;
    int pixels_len;
};

static u8 _IZZ[64] = {
    0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
};

struct s_ht_ary
_create_ht_ary(int count) {
    struct s_ht_ary a = {0, NULL};
    if (count > 0) {
        a.count = count;
        a.v = malloc(sizeof(*a.v) * count); /* free in destroy_jctx */
        memset(a.v, 0, sizeof(*a.v) * count);
    }
    return a;
}

void
_destroy_ht_ary(struct s_ht_tbl *ht) {
    int i;
    for (i=0; i<VLC_MAX_LEN; i++) {
        struct s_ht_ary *a = &ht->ary[i];
        if (a->count > 0)
            free(a->v);
    }
}

struct s_jctx*
_create_jctx(void) {
    struct s_jctx *j = malloc(sizeof(*j));
    if ( j ) {
        memset(j, 0, sizeof(*j));
    }
    return j;
}

void
_destroy_jctx(struct s_jctx *j) {
    if ( j ) {
        int i;
        for (i=0; i<4; i++)
            _destroy_ht_ary( &j->htbl[i] );
        free(j->scan_out);
        free(j->pixels);
        free(j);
    }
}

void
_save_to_ppm(struct s_jctx *j) {
    if ( j->scan_out ) {
        FILE *fp = fopen("export.ppm", "wb");
        assert(fp);
        fprintf(fp, "P%d\n", j->mcu_blocks<=1 ? 5 : 6);
        fprintf(fp, "%d %d\n255\n", j->width, j->height);
        fwrite(j->pixels, 1, j->pixels_len, fp);
        fclose(fp);
        _log(D_INFO, "# Save to export.ppm ok #\n");
    }
}

void
_get_qt_table(struct s_bctx *b, struct s_jctx *j) {
    u16 len = _next_word(b);
    int s=_get_offset(b), e=s+len-2;
    while (s < e) {
        u8 buf = _next_byte(b);
        u8 precision = buf >> 4;
        u8 id = buf & 0xf;
        j->qtbl[id] = _get_ptr(b);
        _log(D_MARKER, "DQT precision:%d id:%d\n", precision, id);
        _skip_bytes(b, 64);
        s = _get_offset(b);
    }
}

const char*
_print_binary(int x, int len) {
    static char b[33] = {0};
    
    u8 offset = len>0 ? (len-1) : 31;
    u32 z = 1<<offset;
    int i=0, start=len;
    while (z > 0) {
        if ( !start ) start = x & z;
        if ( start ) b[i++] = ((x&z)==z) ? '1' : '0';
        z>>=1;
    }
    b[len] = '\0';
    return b;
}

void
_print_ht_table(struct s_jctx *j, int idx) {
    int i, n;
    struct s_ht_tbl *h = &j->htbl[idx];
    printf("## ht table vlc[%d] count %d\n", idx, h->count);
    for (n=0; n<VLC_MAX_LEN; n++) {
        struct s_ht_ary *a = &h->ary[n];
        //printf("vlc len %d, count %d\n", n+1, a->count);
        for (i=0; i<a->count; i++) {
            struct s_ht_vlc *v = &a->v[i];
            printf("\tvlc:%-3d %02x, %s\n", n+1, v->code, _print_binary(v->val, n+1));
        }
    }
}

void
_get_ht_table(struct s_bctx *b, struct s_jctx *j) {
    int i, w, ht_base;
    u16 len=_next_word(b);
    u32 start=_get_offset(b), end=start+len-2;
    while (start < end) {
        u8 buf = _next_byte(b);
        u8 typ_n_id = (buf>>3)|(buf&0xf); /* combine them */
        struct s_ht_tbl *ht = &j->htbl[typ_n_id];
        for (i=0; i<VLC_MAX_LEN; i++) {
            int vlc_count = _next_byte(b);
            ht->count += vlc_count;
            ht->ary[i] = _create_ht_ary( vlc_count );
        }
        for (w=0, ht_base=0; w<VLC_MAX_LEN; w++) {
            struct s_ht_ary *a = &ht->ary[w];
            for (i=0; i<a->count; i++) {
                struct s_ht_vlc *v  = &a->v[i];
                v->code = _next_byte(b);
                v->val = ht_base;
                //_log(D_VERBOSE, "ht %2d, %s\n", w+1, _print_binary(ht_base, w+1));
                ht_base++;
            }
            if (a->count < (w+1)*(w+1))
                ht_base <<= 1;
        }
        j->htbl_count++;
        start = _get_offset(b);
        _log(D_MARKER, "DHT type_n_id %d, count %d\n", typ_n_id, ht->count);
    }
}

void
_decode_frame(struct s_bctx *b, struct s_jctx *j) {
    int i, hmax=0, vmax=0;
    u16 len = _next_word(b);
    u8 P = _next_byte(b);
    j->height = _next_word(b);
    j->width = _next_word(b);
    j->comp_count = _next_byte(b);
    _log(D_MARKER, "Baseline DCT %d, precision %d, %dx%d, comp %d\n",
           len, P, j->width, j->height, j->comp_count);
    for(i=0; i<j->comp_count; i++) {
        u8 buf;
        struct s_jcomp *c = &j->comp[i];
        c->id = _next_byte(b);
        buf = _next_byte(b);
        c->h_samp = buf >> 4;
        c->v_samp = buf & 0xf;
        c->qtbl_id = _next_byte(b);
        assert((c->h_samp & (c->h_samp - 1)) == 0); /* NOP */
        assert((c->v_samp & (c->v_samp - 1)) == 0); /* NOP */
        if (hmax < c->h_samp) hmax = c->h_samp;
        if (vmax < c->v_samp) vmax = c->v_samp;
        //
        _log(D_COEFF, "\tcomp %d, h:v %d:%d, qtbl_id:%d\n", c->id, c->h_samp, c->v_samp, c->qtbl_id);
        if ((c->h_samp!=1) || (c->v_samp!=1)) {
            _log(D_ERROR, "# Unsupported horizontal & vertical sample factor ! #\n");
            _set_eof(b);
            return;
        }
    }
    j->mcu_sizex = hmax << 3;
    j->mcu_sizey = vmax << 3;
    j->h_mcus = (j->width + j->mcu_sizex - 1) / j->mcu_sizex;
    j->v_mcus = (j->height + j->mcu_sizey - 1) / j->mcu_sizey;
    {
        struct s_jcomp *c = &j->comp[0];
        j->mcu_blocks = c->h_samp * c->v_samp + j->comp_count - 1;

        j->scan_len =  j->width * j->mcu_sizey * j->comp_count;
        j->scan_out = (u8*)malloc( j->scan_len );

        j->pixels_len = j->width * j->height * j->comp_count;
        j->pixels = (u8*)malloc( j->pixels_len );
    }
    _log(D_COEFF, "\tmcu, sx:%d sy:%d h:%d v:%d blocks:%d\n",
         j->mcu_sizex, j->mcu_sizey, j->h_mcus, j->v_mcus, j->mcu_blocks);
}

// from nanojpeg.c
u8 _truncate(const s32 x) {
    return (x<0) ? 0 : ((x>0xff) ? 0xff : (u8)x);
}

#define W1 2841
#define W2 2676
#define W3 2408
#define W5 1609
#define W6 1108
#define W7 565

void
_idct_row(s32 *blk) {
    s32 x0, x1, x2, x3, x4, x5, x6, x7, x8;
    if (!((x1 = blk[4] << 11)
        | (x2 = blk[6])
        | (x3 = blk[2])
        | (x4 = blk[1])
        | (x5 = blk[7])
        | (x6 = blk[5])
        | (x7 = blk[3])))
    {
        blk[0] = blk[1] = blk[2] = blk[3] = blk[4] = blk[5] = blk[6] = blk[7] = blk[0] << 3;
        return;
    }
    x0 = (blk[0] << 11) + 128;
    x8 = W7 * (x4 + x5);
    x4 = x8 + (W1 - W7) * x4;
    x5 = x8 - (W1 + W7) * x5;
    x8 = W3 * (x6 + x7);
    x6 = x8 - (W3 - W5) * x6;
    x7 = x8 - (W3 + W5) * x7;
    x8 = x0 + x1;
    x0 -= x1;
    x1 = W6 * (x3 + x2);
    x2 = x1 - (W2 + W6) * x2;
    x3 = x1 + (W2 - W6) * x3;
    x1 = x4 + x6;
    x4 -= x6;
    x6 = x5 + x7;
    x5 -= x7;
    x7 = x8 + x3;
    x8 -= x3;
    x3 = x0 + x2;
    x0 -= x2;
    x2 = (181 * (x4 + x5) + 128) >> 8;
    x4 = (181 * (x4 - x5) + 128) >> 8;
    blk[0] = (x7 + x1) >> 8;
    blk[1] = (x3 + x2) >> 8;
    blk[2] = (x0 + x4) >> 8;
    blk[3] = (x8 + x6) >> 8;
    blk[4] = (x8 - x6) >> 8;
    blk[5] = (x0 - x4) >> 8;
    blk[6] = (x3 - x2) >> 8;
    blk[7] = (x7 - x1) >> 8;
}

void
_idct_col(const s32* blk, u8 *out, int stride) {
    s32 x0, x1, x2, x3, x4, x5, x6, x7, x8;
    if (!((x1 = blk[8*4] << 8)
        | (x2 = blk[8*6])
        | (x3 = blk[8*2])
        | (x4 = blk[8*1])
        | (x5 = blk[8*7])
        | (x6 = blk[8*5])
        | (x7 = blk[8*3])))
    {
        x1 = _truncate(((blk[0] + 32) >> 6) + 128);
        for (x0 = 8;  x0;  --x0) {
            *out = (u8) x1;
            out += stride;
        }
        return;
    }
    x0 = (blk[0] << 8) + 8192;
    x8 = W7 * (x4 + x5) + 4;
    x4 = (x8 + (W1 - W7) * x4) >> 3;
    x5 = (x8 - (W1 + W7) * x5) >> 3;
    x8 = W3 * (x6 + x7) + 4;
    x6 = (x8 - (W3 - W5) * x6) >> 3;
    x7 = (x8 - (W3 + W5) * x7) >> 3;
    x8 = x0 + x1;
    x0 -= x1;
    x1 = W6 * (x3 + x2) + 4;
    x2 = (x1 - (W2 + W6) * x2) >> 3;
    x3 = (x1 + (W2 - W6) * x3) >> 3;
    x1 = x4 + x6;
    x4 -= x6;
    x6 = x5 + x7;
    x5 -= x7;
    x7 = x8 + x3;
    x8 -= x3;
    x3 = x0 + x2;
    x0 -= x2;
    x2 = (181 * (x4 + x5) + 128) >> 8;
    x4 = (181 * (x4 - x5) + 128) >> 8;
    *out = _truncate(((x7 + x1) >> 14) + 128);  out += stride;
    *out = _truncate(((x3 + x2) >> 14) + 128);  out += stride;
    *out = _truncate(((x0 + x4) >> 14) + 128);  out += stride;
    *out = _truncate(((x8 + x6) >> 14) + 128);  out += stride;
    *out = _truncate(((x8 - x6) >> 14) + 128);  out += stride;
    *out = _truncate(((x0 - x4) >> 14) + 128);  out += stride;
    *out = _truncate(((x3 - x2) >> 14) + 128);  out += stride;
    *out = _truncate(((x7 - x1) >> 14) + 128);
}
// end of idct

void
_h1v1_convert_mcu(struct s_jctx *j, int mcu_n) {
    int x, y, obase, pbase;
    u8 *out = j->scan_out;
    u8 *py = j->comp[0].pixels;
    u8 *pcb = j->comp[1].pixels;
    u8 *pcr = j->comp[2].pixels;
    pbase = 0;
    obase = mcu_n * j->mcu_sizex * 3;
    for (y=0; y<j->mcu_sizey; y++) {
        for (x=0; x<j->mcu_sizex; x++) {
            register s32 y = py[pbase+x] << 8;
            register s32 cb = pcb[pbase+x] - 128;
            register s32 cr = pcr[pbase+x] - 128;
            out[obase+x*3  ] = _truncate((y +            359 * cr + 128) >> 8);
            out[obase+x*3+1] = _truncate((y -  88 * cb - 183 * cr + 128) >> 8);
            out[obase+x*3+2] = _truncate((y + 454 * cb            + 128) >> 8);            
        }
        pbase += DCTSIZE;
        obase += j->width * 3;
    }
    //_dump_buf(out, j->width*3);
}

void
_grayscale_convert_mcu(struct s_jctx *j, int mcu_n) {
    int y, obase, pbase;
    u8 *out = j->scan_out;
    u8 *py = j->comp[0].pixels;
    pbase = 0;
    obase = mcu_n * j->mcu_sizex;
    for (y=0; y<j->mcu_sizey; y++) {
        memcpy(&out[obase], &py[pbase], DCTSIZE);
        pbase += DCTSIZE;
        obase += j->width;
    }
}

int
_check_vlc_in_ht(struct s_bctx *b, struct s_ht_tbl *ht, u8 *code) {
    int i, n, val;
    struct s_ht_ary *a = NULL;
    for (n=1; n<=VLC_MAX_LEN; n++) {
        val = _bits_try(b, n);
        a = &ht->ary[n-1];
        assert(a);
        for (i=0; i<a->count; i++) {
            struct s_ht_vlc *v = &a->v[i];
            if (val == v->val) {
                _log(D_VERBOSE, "src:%x code:%x bits:%d\n", val<<(16-n) , v->code, n);
                _bits_skip(b, n);
                if ( code ) { *code = (u8)v->code; }
                n = v->code & 0xf;
                if ( !n ) { _log(D_VERBOSE, "-- bits 0\n"); return 0; }
                val = _bits_read(b, n);
                if (val < (1<<(n-1))) { val += (int)(~(1<<n)) + 2; }
                _log(D_VERBOSE, "decode val:%x, bits %d\n", val&0xff, n);
                return val;
            }
        }
    }
    _log(D_ERROR, "# Fail to decode huff at %d, val %s #\n", _get_offset(b), _print_binary(val, 16));
    _dump_buf(_get_ptr(b), DCTSIZE);
    assert(0);
    return 0;
}

// get DC/AC and de-quant, invert zig-zag
static void
_decode_block(struct s_bctx *b, struct s_jctx *j, int comp_id) {
    int i, ai, val;
    struct s_ht_tbl *htbl = NULL;
    struct s_jcomp *c = &j->comp[comp_id];
    const u8 *qtbl = j->qtbl[c->qtbl_id];
    assert( qtbl );

    _log(D_VERBOSE, "decode comp %d, qtbl_id:%d ht_dc:%d ht_ac:%d\n",
         comp_id, c->qtbl_id, c->ht_dc_id, c->ht_ac_id);

    memset(c->vec, 0, DCTSIZE2*sizeof(c->vec[0]));

    // get DC
    htbl = &j->htbl[c->ht_dc_id];
    assert(htbl);
    val = _check_vlc_in_ht(b, htbl, NULL);
    c->dc += val;
    c->vec[0] = c->dc * qtbl[0];
    //_log(D_VERBOSE, "DC %d\n", c->dc);
    
    // get AC
    htbl = &j->htbl[c->ht_ac_id];
    assert(htbl);
    for (ai=1; ai<64; ai++) {
        u8 code = 0;
        val = _check_vlc_in_ht(b, htbl, &code);
        if ( !code ) { _log(D_VERBOSE, "-- EOB\n"); break; }    /* EOB */
        else {
            ai += (code >> 4);
            c->vec[(s32) _IZZ[ai] ] = val * qtbl[ai]; /* dequant */
        }
    }

    //_dump_buf((u8*)c->vec);
    
    // idct
    for (i=0; i<DCTSIZE2; i+=DCTSIZE)
        _idct_row( &c->vec[i] );
    for (i=0; i<DCTSIZE; i++)
        _idct_col( &c->vec[i], &c->pixels[i], DCTSIZE);

    //_dump_buf(c->pixels);
}

void
_decode_scan(struct s_bctx *b, struct s_jctx *j) {
    int i;
    u16 len = _next_word(b);
    u8 comp = _next_byte(b);
    _log(D_MARKER, "Scan header %d, %d\n", len, comp);
    for (i=0; i<comp; i++) {
        struct s_jcomp *c = &j->comp[i];
        u8 id = _next_byte(b);
        u8 buf = _next_byte(b);
        c->ht_dc_id = buf >> 4;
        c->ht_ac_id = (buf & 1) | 2;
        _log(D_COEFF, "\tcomp id %d, dc:%d, ac:%d\n", id, c->ht_dc_id, c->ht_ac_id);
    }
    {
        u8 ss = _next_byte(b);
        u8 se = _next_byte(b);
        u8 buf = _next_byte(b);
        _log(D_COEFF, "\tss %d, se %d, ah ai %x\n", ss, se, buf);
        assert(se==63);
    }
    {
        int x, y;
        for (y=0; y<j->v_mcus; y++) {
            for (x=0; x<j->h_mcus; x++) {
                // decode MCU
                for (i=0; i<j->mcu_blocks; i++) {
                    _decode_block(b, j, i);
                }
                switch ( j->mcu_blocks ) {
                    case 1: _grayscale_convert_mcu( j, x ); break;
                    case 3: _h1v1_convert_mcu( j, x ); break; // YUV to RGB
                }

                // restart every comp's dc
                if (j->restintv && !(--j->restintv_cnt)) {
                    u16 RSTx = _next_word(b);
                    _bits_clear(b);
                    if (RSTx == M_EOI) {
                        _skip_bytes(b, -2);
                        goto end_scan_line;
                    }
                    _log(D_VERBOSE, "RST meets %4x, %04x\n", RSTx, j->restintv_next);
                    if (((RSTx&0xfff8)!=0xffd0) || ((RSTx&0x7)!=j->restintv_next)) {
                        _dump_buf(_get_ptr(b), DCTSIZE);
                        assert(0);
                    }
                    j->restintv_next = (RSTx + 1) & 0x7;
                    j->restintv_cnt = j->restintv;
                    for (i=0; i<j->comp_count; i++) {
                        j->comp[i].dc = 0;
                    }
                }
            }
        end_scan_line:
            memcpy(&j->pixels[y*j->scan_len], j->scan_out, j->scan_len);
        }
    }
}

void
_decode_dri(struct s_bctx *b, struct s_jctx *j) {
    u16 lr = _next_word(b);
    u16 ri = _next_word(b);
    _log(D_MARKER, "DRI info %d, %x\n", lr, ri);
    j->restintv = ri;
    j->restintv_cnt = ri;
    j->restintv_next = 0;
}

int
_skip_segment(struct s_bctx *b) {
    u16 len = _next_word(b);
    _skip_bytes(b, len - 2);
    return len;
}



int
_decode(struct s_bctx *b, struct s_jctx *j) {
    while ( !_is_eof(b) ) {
        u16 marker = _next_word(b);
        switch ( marker ) {
            case M_SOI: _log(D_MARKER, "SOI\n"); break;
            case M_EOI: _log(D_MARKER, "EOI\n"); break;
            case M_DQT: _get_qt_table(b, j); break;
            case M_SOF0: _decode_frame(b, j); break;
            case M_DRI: _decode_dri(b, j); break;
            case M_DHT: _get_ht_table(b, j); break;
            case M_SOS: _decode_scan(b, j); break;
            default:
                if (marker>=M_APP0 && marker<=M_APPn) {
                    _log(D_MARKER, "APP segment length %d\n", _skip_segment(b));
                }
                else if (marker == M_COM) {
                    _log(D_MARKER, "COM segment length %d\n", _skip_segment(b));
                }
                else {
                    _log(D_ERROR, "# Unknow marker %x ! #\n", marker);
                    return 0;
                }
                break;
        }
    }
    return 1;
}

int
main(int argc, char *argv[])
{
    long length = 0;
    u8 *content = NULL;

    if (argc != 2) {
        _log(D_INFO, "%s FILE.JPG\n", argv[0]);
        return 0;
    }

    if ( _get_file_content( argv[1], &content, &length ) ) {
        struct s_bctx *b = _create_bctx( content, (u32)length );
        struct s_jctx *j = _create_jctx();

        // decoder will malloc buffer in j->pixles
        if ( _decode(b, j) ) {
            _save_to_ppm( j ); 
        }
        else {
            _log(D_ERROR, "Fail to decode !!!\n");
        }

        _destroy_jctx( j );
        _destroy_bctx( b );
        _free_file_content( content );
    }

    return 0;
}
