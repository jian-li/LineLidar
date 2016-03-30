#include "pointmatcher/PointMatcher.h"
#include <sys/time.h>

namespace LineLidar
{
	class 2dslam
	{
	public:
		2dslam();
		~2dslam();
		void setcurrent_scan(DP* current_scan);
		void setref_scan(DP* ref_scan);
		void getnew_scan(DP* new_scan);
		void do_icp();
	private:
		bool newframe;
		bool first_scan;
		bool transform_calculated;
		PM::TransformationParameters transform;
		DP* ref_scan;
		DP* current_scan;
		PointMatcher<float> pointmatcher;
		PM:icp icp_al;
	};
}