
namespace LineLidar
{
	2dslam::2dslam()
	{
		this->newframe = false;
		this->firstscan = true;
		this->transform_calculated = false;
		this->icp_al.setDefault();
	}

	2dslam::~2dslam()
	{

	}

	2dslam::setref_scan(DP* mref_scan)
	{
		this->ref_scan = mref_scan;
	}

	2dslam::setcurrent_scan(DP* mcurrent_scan)
	{
		this->current_scan = mcurrent_scan;
	}

	2dslam:getnew_scan(DP* new_scan)
	{
		if(this->firstscan)
		{
			this->setcurrent_scan(new_scan);
		}
		else
		{
			this->setref_scan(mcurrent_scan);
			this->setcurrent_scan(new_scan);
			this->newframe = true;
		}
	}


	2dslam:do_icp()
	{
		timeval starttime,endtime;
      	gettimeofday(&starttime,0);

      	transform = icp_al(this->current_scan, this->ref_scan);

      	// Transform data to express it in ref
		DP data_out(*this->current_scan);
		icp_al.transformations.apply(*this->current_scan, transform);

		gettimeofday(&endtime,0);
      	double timeuse = 1000000*(endtime.tv_sec - starttime.tv_sec) + endtime.tv_usec - starttime.tv_usec;
	    cout << timeuse/1000 << endl;
	}
}