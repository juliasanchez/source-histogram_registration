void get_error_theta(std::vector<std::vector<float>>& hist1, std::vector<std::vector<float>>& hist2, std::vector<float>& error_theta, int *rotation_theta)
{
    int N_hist=hist1.size();
    float temp=2000000000;
    int p=0;
    std::vector<std::vector<float>> hist11(N_hist,std::vector<float>(2*N_hist,0.0));

    for (int k=0; k<N_hist; k++)
    {
        //turn object of delta_theta

        for (int n=0; n<N_hist; n++)
        {
            for (int m=0; m<2*N_hist; m++)
            {
                if(m+k<N_hist*2)
                {
                    hist11[n][m+k]=hist1[n][m];
                    p=0;
                }
                else
                {
                    hist11[n][p]=hist1[n][m];
                    p++;
                }
            }
        }

        //compute error and get minimum

        for (int n=0; n<N_hist; n++)
        {
            for (int m=0; m<2*N_hist; m++)
            {
                error_theta[k]=error_theta[k]+(hist11[n][m]-hist2[n][m])*(hist11[n][m]-hist2[n][m]);
            }
        }
        if (temp>error_theta[k])
        {
            temp=error_theta[k];
            *rotation_theta=k;
        }
    }
}
