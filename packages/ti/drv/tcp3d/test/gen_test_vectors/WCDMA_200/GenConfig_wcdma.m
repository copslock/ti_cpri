rand('seed', 040511)

sw0 = [ 48 64 96 128 ];

fidL = fopen('config_list.cfg', 'w');

for ind = 1:180
    mode = 3;

    frmSizeInd = round(40 + rand*(5114-40));
    fprintf(fidL, '%s\n', sprintf('config_%03d.cfg',ind));
    
    fid = fopen(sprintf('config_%03d.cfg',ind), 'w');
        fprintf(fid, 'Coding_standard  = %d\n', mode);
        fprintf(fid, 'Frame_size_index = %d\n', frmSizeInd);
        fprintf(fid, 'Max_number_of_turbo_iterations = 8\n');
        fprintf(fid, 'Min_number_of_turbo_iterations = 1\n');
        fprintf(fid, 'Max_star_enable = 0\n');
        fprintf(fid, 'Max_star_threshold = 4\n');
        fprintf(fid, 'Max_star_value     = 2\n');
        fprintf(fid, 'tcp3_extrScaleEn = 1\n');
        fprintf(fid, 'Extrinsic_scales = 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 \n');
        fprintf(fid, 'tcp3_SW0_length = %d\n', sw0(1+fix(rand*3)));
        fprintf(fid, 'tcp3_stopSel = 0\n');
        fprintf(fid, 'tcp3_SNR_Report = 1\n');
        fprintf(fid, 'tcp3_SNR_stopVal = 14\n');
        fprintf(fid, 'tcp3_intlvGenEn = 1\n');
        fprintf(fid, 'tcp3_softOutBitFormat = 1\n');
        fprintf(fid, 'tcp3_lteCrcInitSel = 0\n');
        fprintf(fid, 'tcp3_lteCrcIterPass = 1\n');
        fprintf(fid, 'tcp3_outStatusReadEn = %d\n', round(rand*1));
        fprintf(fid, 'tcp3_softOutBitsReadEn = %d\n', round(rand*1));
        fprintf(fid, 'Save_intermediate_data    = 1\n');
        fprintf(fid, 'Minimum_number_of_FEC_blocks = 1\n');
        fprintf(fid, 'Maximum_number_of_FEC_blocks = 1\n');
        fprintf(fid, 'Snr_increment_step = 0\n');
        fprintf(fid, 'Frame_error_rate_limit = -4\n');
        fprintf(fid, 'Snr_init_value = %d\n',round(rand*7)-3);
        fprintf(fid, 'Add_noise = 1\n');
        seed = round(rand*2^25);
        if(seed<1000)
            seed = seed+1000;
        end
        fprintf(fid, 'c_model_seed = %d\n', seed);
        fprintf(fid, 'Bit_width_of_integer_part = 4\n');
        fprintf(fid, 'Bit_width_of_fractional_part = 2\n');
        fprintf(fid, 'Minimum_number_of_frame_errors = 0\n');
        fprintf(fid, 'Store_info_bits_to_file = 1\n');
        fprintf(fid, 'Load_info_bits_from_file = 0\n');
        fprintf(fid, 'Info_bits_file_name = infobits_file.txt\n');
        fprintf(fid, 'Info_bits_file_includes_CRC= 1\n');
        fprintf(fid, 'Initial_process_index =  0\n');
        fprintf(fid, 'Store_coded_bits_to_file = 0\n');
        fprintf(fid, 'Coded_bits_file_name = codedbits_file.txt\n');
    fclose(fid);
    ind = ind + 1;
end
fclose(fidL);
