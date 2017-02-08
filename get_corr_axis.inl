void get_corr_axis(std::vector<float>& hist1_axisi, std::vector<float>& hist2_axisi, std::vector<float>& error_axis, int *translation_axis)
{
    int p=0;
    float temp=0;
    int N_hist_axis= hist1_axisi.size();
    std::vector<float> hist11_axis(3*N_hist_axis-2, 0.0);
    std::vector<float> hist21_axis(3*N_hist_axis-2, 0.0);

    for (int m=N_hist_axis-1; m<2*N_hist_axis-1; m++)
    {
        hist21_axis[m]=hist2_axisi[p];
        p++;
    }

    for (int k=0; k<2*N_hist_axis-1; k++)
    {
        //move object of delta_axis

        if(k!=0)
        {
           hist11_axis[k-1]=0;
        }
        for (int m=0; m<N_hist_axis; m++)
        {
            hist11_axis[m+k]=hist1_axisi[m];
        }

        //compute error and get minimum

        for (int m=0; m<3*N_hist_axis-2; m++)
        {
            error_axis[k]=error_axis[k]+hist11_axis[m]*hist21_axis[m];
        }

        if (temp<error_axis[k])
        {
            temp=error_axis[k];
            *translation_axis=k;
        }
    }

for (int m=N_hist_axis; m<2*N_hist_axis; m++)
{
    hist11_axis[m]=0;
}
for (int m=0; m<N_hist_axis; m++)
{
    if(*translation_axis<=N_hist_axis)
    {hist11_axis[m+*translation_axis]=hist1_axisi[m]/ *translation_axis;}
    else
    {hist11_axis[m+*translation_axis]=hist1_axisi[m]/(2*N_hist_axis-*translation_axis);}
}

save_vector(hist11_axis, "hist11_axis_for_translation.csv");

}
