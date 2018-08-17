#ifndef G2O_TYPES_H
#define G2O_TYPES_H
#include "lmars/common_include.h"
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace lmars{
	class EdgeProjectXYZ2UVPoseOnly:public g2o::BaseUnaryEdge<2,Eigen::Vector2d,g2o::VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		virtual void computeError();
		virtual void linearizeOplus();
		virtual bool read( std::istream& in ){}
		virtual bool write(std::ostream& os) const {};
		
		Vector3d point_;
		Camera* camera_;
	};
}



#endif