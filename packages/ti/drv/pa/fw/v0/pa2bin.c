#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pa.h"
#include "pdsp_ver.h"
#include "paconst.c"

#define DEVICE_PA_NUM_PDSPS            (6U)
#define PA_PDSP_CONST_NUM_REG        (4U)
#define PASS_VER_STR_LEN            (16U)

typedef uint32_t u32;

#include "pdsp_blob_hdr.h"

static char *versions[DEVICE_PA_NUM_PDSPS] = {
    PASS_VERSION_STR,
    PASS_VERSION_STR,
    PASS_VERSION_STR,
    PASS_VERSION_STR,
    PASS_VERSION_STR,
    PASS_VERSION_STR
};
static char *pdsp_in_file_names[DEVICE_PA_NUM_PDSPS] = {
    "classify1_0.bib",
    "classify1_1.bib",
    "classify1_2.bib",
    "classify2.bib",
    "pam.bib",
    "pam.bib",
};

static char *pdsp_out_file_names[DEVICE_PA_NUM_PDSPS] = {
    "ks2_pa_pdsp0_classify1",
    "ks2_pa_pdsp1_classify1",
    "ks2_pa_pdsp2_classify1",
    "ks2_pa_pdsp3_classify2",
    "ks2_pa_pdsp4_pam",
    "ks2_pa_pdsp5_pam",
};

int main(int argc, char **argv)
{
    unsigned long b_sz, total = 0;
    struct pa_pdsp_firmware pdsp;
    char in_file_name[128];
    char out_file_name[128];
    char buf[1024];
    char *pc;
    FILE *outfp, *infp;
    int j, i;
    int size;

    for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
        if (!pdsp_in_file_names[i])
            continue;
        bzero(in_file_name, sizeof(in_file_name));
        bzero(out_file_name, sizeof(out_file_name));
        strcpy(in_file_name, (char *)pdsp_in_file_names[i]);
        strcpy(out_file_name, (char *)pdsp_out_file_names[i]);
        pc = &out_file_name[strlen(out_file_name)];

        strcat(pc, ".bin");
        outfp=fopen(out_file_name,"wb");
        infp=fopen(in_file_name,"rb");
        printf("input file: %s\n",in_file_name);
        printf("output file: %s\n",out_file_name);
        if (!outfp)
        {
            printf("Unable to open output file! %s", out_file_name);
            exit(1);
        }

        if (!infp)
        {
            printf("Unable to open input file! %s", in_file_name);
            fclose(outfp);
            exit(2);
        }

        strncpy((char *)&pdsp.version[0], (char *)versions[i], PASS_VER_STR_LEN);
        memcpy(&pdsp.constants[0], &pap_pdsp_const_reg_map[i][0], sizeof(u32) * PA_PDSP_CONST_NUM_REG);
        fwrite(&pdsp, sizeof(pdsp), 1, outfp);
        while(1) {
            size = fread(buf, 1, 1024, infp);
            if (size <= 0)
                break;
            fwrite(buf, size, 1, outfp);
        }
        fclose(outfp);
        fclose(infp);
    }
}
