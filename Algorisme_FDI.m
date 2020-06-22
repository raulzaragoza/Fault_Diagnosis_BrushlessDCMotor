
clear
close all

% *******ABRUPTES*********
% load dades_Vdc_Fault_abrup
% load dades_R_Fault_abrup
% load dades_L_Fault_abrup
% load dades_isensor_Fault_abrup
% load dades_stuckpos_Fault_abrup
% load dades_B_Fault_abrup
% load dades_J_Fault_abrup
% load dades_Load_Fault_abrup 

% *******INCIPIENTS*********
% load dades_Vdc_Fault_incip
% load dades_R_Fault_incip
% load dades_L_Fault_incip 
% load dades_isensor_Fault_incip
% load dades_stuckpos_Fault_incip
% load dades_B_Fault_incip
% load dades_J_Fault_incip  
% load dades_Load_Fault_incip  

Tsd=0.0001;
t=0:Tsd:(length(r1)-1)*Tsd;
t=t';

k=1;
L=length(r1);
while and(k<=L,sum(fi)==0)
     fi=zeros(4,1);
    
    if abs(r1(k))>=sigma_1d
         fi(1)=1;
    end
    
    if abs(r2(k))>=sigma_2d
         fi(2)=1;
    end
    
    if abs(r3(k))>=sigma_3d
         fi(3)=1;
    end
    
    if abs(r4(k))>=sigma_4d
         fi(4)=1;
    end
    
   
    
    k=k+1;
    
end

kfault=k-1;
fh=ones(4,1); 
k=k-1;

while and(k<=L,sum(fh)>1)

    fi=zeros(4,1);
    
    if abs(r1(k))>=sigma_1d
         fi(1)=1;
         fh(1)=0;
    end
    
    if abs(r2(k))>=sigma_2d
         fi(2)=1;
         fh(2)=0;
    end
    
    if abs(r3(k))>=sigma_3d
        fi(3)=1; 
        fh(3)=0;
    end
    
    if abs(r4(k))>=sigma_4d
        fi(4)=1; 
        fh(4)=0;
    end
    
    
    
    k=k+1;
end

faultype=[1,2,3,4]*fh
kisol=k-1;

switch faultype
    case 1 
        %% Particularitzem per fh(1)=> Fallada en B (proporcional a w_m), J (proporcional a dw/dt), en V;

        %filtrem senyals d'inici

        r2_f=zeros(size(r2));
        w_m_f=zeros(size(r2));
        d_wm_f=zeros(size(r2));
        d_wm=zeros(size(r2));
        d_wm(2:end)=(w_m(2:end)-w_m(1:end-1))/Tsd;
        
        
        %Filtre Butterworth
        fmax=1/Tsd;
        [b,a] = butter(2,10/fmax); %Frec tall =1Hz
        r2_f=filter(b,a,r2);
        w_m_f=filter(b,a,w_m);
        d_wm_f=filter(b,a,d_wm);
        
        figure
        subplot (3,1,1)
        plot (t,r2)
        hold on
        plot (t,r2_f)
        title ('Residu 2 filtrat vs no filtrat')
        ylabel ('\omega[rad/s]')
        subplot (3,1,2)
        plot (t,w_m)
        hold on
        plot (t,w_m_f)
        title ('Velocitat filtrada vs no filtrada')
        ylabel ('\omega[rad/s]')
        subplot (3,1,3)
        plot (t,d_wm)
        hold on
        plot (t,d_wm_f) 
        title ('Derivada de la velocitat filtrada vs no filtrada')
        ylabel ('d\omega/dt[rad/s^2]')
        xlabel ('Time[s]')
      
        W_c=1/Tsd; %Finestra Correlacio

        mech_isol=0;
        while mech_isol==0
        [corr_r2_B,corr_r2_J,r2_Wc,S_f_B,S_f_J]=correlation(r2_f,w_m_f,d_wm_f,W_c,kisol);
       
        t2=0:Tsd:(length(r2_Wc)-1)*Tsd;
        t2=t2'+(kisol)*Tsd;
        figure
        subplot (3,1,1)
        plot (t2,r2_Wc)
        title ('Residu 2 en la finestra de correlació')
        ylabel ('\omega[rad/s]')
        subplot (3,1,2)
        plot (t2,S_f_B)
        title ('Velocitat filtrada en la finestra de correlació')
        ylabel ('\omega[rad/s]')
        subplot (3,1,3)
        plot (t2,S_f_J)
        title ('Derivada de la velocitat filtrada en la finestra de correlació')
        ylabel ('d\omega/dt[rad/s^2]')
        xlabel ('Time[s]')
         
        % Fault isolation Final part
        
        if max ([corr_r2_B,corr_r2_J])>0.75 && max ([corr_r2_B,corr_r2_J])<0.9
            W_c=W_c+10000;
        else
            if max ([corr_r2_B,corr_r2_J]) < 0.75
                disp ('Tload fault')
                mech_isol=1;
            else
                if corr_r2_B>corr_r2_J
                disp ('B fault')
                mech_isol=1;
                else
                disp ('J fault')
                mech_isol=1;
                end
            end
        end
        end
    case 2
        %% Particularitzem per fh(2)=> Fallada en R (proporcional a i_m), L (proporcional adi/dt), en V;

        %filtrem senyals d'inici

        r1_f=zeros(size(r1));
        i_m_f=zeros(size(r1));
        d_im=zeros(size(r1));
        d_im(2:end)=(i_m(2:end)-i_m(1:end-1))/Tsd;
        d_im_f=zeros(size(r1));

        
        %Filtre Butterworth
        fmax=1/Tsd;
        [b,a] = butter(2,10/fmax); %Frec tall =1Hz
        r1_f=filter(b,a,r1);
        i_m_f=filter(b,a,i_m);
        d_im_f=filter(b,a,d_im);
        
        figure
        subplot (3,1,1)
        plot (t,r1)
        hold on
        plot (t,r1_f)
        title ('Residu 1 filtrat vs no filtrat')
        ylabel ('i[A]')
        subplot (3,1,2)
        plot (t,i_m)
        hold on
        plot (t,i_m_f)
        title ('Corrent filtrat vs no filtrat')
        ylabel ('i[A]')
        subplot (3,1,3)
        plot (t,d_im)
        hold on
        plot (t,d_im_f)
        title ('Derivada del corrent filtrat vs no filtrat')
        ylabel ('di/dt[A/s]')
        xlabel ('Time[s]')
        
        W_c=1/Tsd; %Finestra Correlacio
        elect_isol=0;

        while elect_isol==0
        [corr_r1_R,corr_r1_L,r1_Wc,S_f_R,S_f_L]=correlation(r1_f,i_m_f,d_im_f,W_c,kisol);
        t2=0:Tsd:(length(r1_Wc)-1)*Tsd;
        t2=t2'+kisol*Tsd;
        figure
        subplot (3,1,1)
        plot (t2,r1_Wc)
        title ('Residu 1 en la finestra de correlació')
        ylabel ('i[A]')
        subplot (3,1,2)
        plot (t2,S_f_R)
        title ('Corrent filtrat en la finestra de correlació')
        ylabel ('i[A]')
        subplot (3,1,3)
        plot (t2,S_f_L)
        title ('Derivada del corrent filtrat en la finestra de correlació')
        ylabel ('di/dt[A/s]')
        xlabel ('Time[s]')
        
        % Fault isolation Final part
        if max ([corr_r1_R,corr_r1_L])>0.25 && max ([corr_r1_R,corr_r1_L])<0.9
            W_c=W_c+10000;
        else
            if max ([corr_r1_R,corr_r1_L]) < 0.25
                disp ('Vdc fault')
                elect_isol=1;
            else
                if corr_r1_R>corr_r1_L
                    disp ('R fault')
                    elect_isol=1;
                else
                    disp ('L fault')
                    elect_isol=1;
                end
            end
        end
        end
    case 3
        disp ('Stuck position fault')
    case 4 
        disp ('Current sensor fault')
    otherwise
        warning ('Unexpected/Unknown fault')
end