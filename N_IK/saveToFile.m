function saveToFile(D_current, motion, str_unit_chosen, inverse_chosen, alpha, option)
    
    % check if it is the 3DoF robot
    if (length(D_current) == 2)
        fileID_x = fopen(strcat(motion, "/X_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_x,'%16.12f\r\n',D_current(1));
        fclose(fileID_x);

        fileID_y = fopen(strcat(motion, "/Y_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_y,'%16.12f\r\n',D_current(2));
        fclose(fileID_y);
        
    elseif (length(D_current) == 3)
        fileID_x = fopen(strcat(motion, "/X_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_x,'%16.12f\r\n',D_current(1));
        fclose(fileID_x);

        fileID_y = fopen(strcat(motion, "/Y_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_y,'%16.12f\r\n',D_current(2));
        fclose(fileID_y);
        
        fileID_z = fopen(strcat(motion, "/Z_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_z,'%16.12f\r\n',D_current(3));
        fclose(fileID_z);
        
    else
        
        fileID_x = fopen(strcat(motion, "/X_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_x,'%16.12f\r\n',D_current(1));
        fclose(fileID_x);

        fileID_y = fopen(strcat(motion, "/Y_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_y,'%16.12f\r\n',D_current(2));
        fclose(fileID_y);

        fileID_z = fopen(strcat(motion, "/Z_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_z,'%16.12f\r\n',D_current(3));
        fclose(fileID_z);

        fileID_rr = fopen(strcat(motion, "/Roll_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_rr,'%16.12f\r\n',D_current(4));
        fclose(fileID_rr);

        fileID_rp = fopen(strcat(motion, "/Pitch_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_rp,'%16.12f\r\n',D_current(5));
        fclose(fileID_rp);

        fileID_ry = fopen(strcat(motion, "/Yaw_", str_unit_chosen, "_", inverse_chosen, "_alpha_", num2str(alpha), ".txt"), option);
        fprintf(fileID_ry,'%16.12f\r\n',D_current(6));
        fclose(fileID_ry);
        
    end

end