void get_error_phi(std::vector<float>& hist1_phi, std::vector<float>& hist2_phi, std::vector<float>& error_phi, int *rotation_phi)
{
    int N_hist=hist1_phi.size();
    float temp=2000000000;
    int p=0;
    std::vector<float> hist11_phi(N_hist, 0.0);

    for (int k=0; k<N_hist; k++)
    {
        //turn object of delta_phi

        for (int m=0; m<N_hist; m++)
        {
            if(m+k<N_hist)
            {
                hist11_phi[m+k]=hist1_phi[m];
                p=0;
            }
            else
            {
                hist11_phi[p]=hist1_phi[m];
                p++;
            }
        }

        error_phi[k]=0;

        //compute error and get minimum

        for (int m=0; m<N_hist; m++)
        {
            error_phi[k]=error_phi[k]+(hist11_phi[m]-hist2_phi[m])*(hist11_phi[m]-hist2_phi[m]);
        }

        if (temp>error_phi[k])
        {
            temp=error_phi[k];
            *rotation_phi=k;
        }
    }
}
