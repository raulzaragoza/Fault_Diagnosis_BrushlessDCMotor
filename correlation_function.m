function [corr_a,corr_b,r_Wc,S_f_a,S_f_b] = correlation(r_f,m_f,d_m_f,W_c,samp)
%CORRELATION
%   This function aims calculating two correlations taking residual and a measuring signal or its differencial.  
r_Wc =r_f(samp:samp+W_c);
S_f_a=-1*m_f(samp:samp+W_c);
S_f_b=-1*d_m_f(samp:samp+W_c);

corr_a=sum(r_Wc.*S_f_a)/(norm(r_Wc,2)*norm(S_f_a,2));
corr_b=sum(r_Wc.*S_f_b)/(norm(r_Wc,2)*norm(S_f_b,2));

end

