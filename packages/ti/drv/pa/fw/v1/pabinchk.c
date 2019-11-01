#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pa.h"
#include "pdsp_ver.h"
#include "paconst.c"

#define DEVICE_PA_NUM_PDSPS            (15U)
#define PA_PDSP_CONST_NUM_REG          (32U)
#define PASS_VER_STR_LEN               (16U)

#include "pdsp_blob_hdr.h"

static char *versions[DEVICE_PA_NUM_PDSPS] = {
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR,
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR, 
    PASS_VERSION_STR    
};
static char *pdsp_in_file_names[DEVICE_PA_NUM_PDSPS] = {
    "in0_pdsp0.bib",    /*  0 */
    "in0_pdsp1.bib",    /*  1 */
    "in1_pdsp0.bib",    /*  2 */
    "in1_pdsp1.bib",    /*  3 */
    "in2_pdsp0.bib",    /*  4 */
    "in3_pdsp0.bib",    /*  5 */
    "in4_pdsp0.bib",    /*  6 */
    "in4_pdsp1.bib",    /*  7 */
    "post_pdsp0.bib",   /*  8 */
    "post_pdsp1.bib",   /*  9 */
    "eg0_pdsp0.bib",    /* 10 */
    "eg0_pdsp1.bib",    /* 11 */
    "eg0_pdsp2.bib",    /* 12 */
    "eg1_pdsp0.bib",    /* 13 */
    "eg2_pdsp0.bib",    /* 14 */
};

static char *pdsp_out_file_names[DEVICE_PA_NUM_PDSPS] = {
    "ks2_pa_in0_pdsp0",    /*  0 */
    "ks2_pa_in0_pdsp1",    /*  1 */
    "ks2_pa_in1_pdsp0",    /*  2 */
    "ks2_pa_in1_pdsp1",    /*  3 */
    "ks2_pa_in2_pdsp0",    /*  4 */
    "ks2_pa_in3_pdsp0",    /*  5 */
    "ks2_pa_in4_pdsp0",    /*  6 */
    "ks2_pa_in4_pdsp1",    /*  7 */
    "ks2_pa_post_pdsp0",   /*  8 */
    "ks2_pa_post_pdsp1",   /*  9 */
    "ks2_pa_eg0_pdsp0",    /* 10 */
    "ks2_pa_eg0_pdsp1",    /* 11 */
    "ks2_pa_eg0_pdsp2",    /* 12 */
    "ks2_pa_eg1_pdsp0",    /* 13 */
    "ks2_pa_eg2_pdsp0",    /* 14 */
};

int main(int argc, char **argv)
{
    unsigned long b_sz, total = 0;
    struct pa_pdsp_firmware pdsp;
    char in_file_name[128];
    char out_file_name[128];
    char buf_in[1024];
    char buf_out[1024];
    char *pc;
    FILE *outfp, *infp;
    int j, i;
    int size, mismatch = 0;

    for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
        if (!pdsp_in_file_names[i])
            continue;
        bzero(in_file_name, sizeof(in_file_name));
        bzero(out_file_name, sizeof(out_file_name));
        strcpy(in_file_name, (char *)pdsp_in_file_names[i]);
        strcpy(out_file_name, (char *)pdsp_out_file_names[i]);
        pc = &out_file_name[strlen(out_file_name)];

        strcat(pc, ".bin");
        outfp=fopen(out_file_name,"rb");
        infp=fopen(in_file_name,"rb");

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

        /* Check the version */
        fread(&pdsp, sizeof(pdsp), 1, outfp);
        if ( 0 != strncmp((char *)&pdsp.version[0], (char *)versions[i], PASS_VER_STR_LEN))
        {
            mismatch = 1;
        }
        
        /* Check the constant */
        if ( 0 != memcmp(&pdsp.constants[0], &pap_pdsp_const_reg_map[i][0], sizeof(u32) * PA_PDSP_CONST_NUM_REG))
        {
            mismatch = 2;
        }
        
        while(mismatch == 0) {
            size = fread(buf_out, 1, 1024, outfp);
            if (size <= 0)
                break;

            size = fread(buf_in, 1, 1024, infp);
            if (size <= 0)
                break;

            if ( 0 != memcmp(buf_in, buf_out, size))
            {
                mismatch = 3;
            }
        }
        fclose(outfp);
        fclose(infp);

        if (mismatch != 0)
           break;
    }

    switch (mismatch)
    {

        case 3:
          printf ("blob mismatch noticed on writing %s.bin file \n", pdsp_out_file_names[i]);
          printf ("firmware write failed ....recheck \n");
          break;

        case 2:
          printf ("constants mismatch noticed on writing %s.bin file \n", pdsp_out_file_names[i]);
          printf ("firmware write failed ....recheck \n");
          break;

        case 1:
          printf ("version mismatch noticed on writing  %s.bin file \n", pdsp_out_file_names[i]);
          printf ("firmware write failed ....recheck \n");
          break;

        default:
          printf ("all firmware written successfully...all matched \n");
          break;
    }
}
