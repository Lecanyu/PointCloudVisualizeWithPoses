#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Eigen>


struct FramedTransformation {
	int frame1_;
	int frame2_;
	Eigen::Matrix4d transformation_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	FramedTransformation()
		:frame1_(-1), frame2_(-1), transformation_(Eigen::Matrix4d::Identity())
	{}
	FramedTransformation(int frame1, int frame2, const Eigen::Matrix4d& t)
		: frame1_(frame1), frame2_(frame2), transformation_(t)
	{}
};



struct RGBDTrajectory {
	std::vector< FramedTransformation, Eigen::aligned_allocator<FramedTransformation>> data_;

	void LoadFromFile(std::string filename) {
		data_.clear();
		int frame1, frame2;
		Eigen::Matrix4d trans;
		FILE * f = fopen(filename.c_str(), "r");
		if (f != NULL) {
			char buffer[1024];
			while (fgets(buffer, 1024, f) != NULL) {
				if (strlen(buffer) > 0 && buffer[0] != '#') {
					sscanf(buffer, "%d %d", &frame1, &frame2);
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(0, 0), &trans(0, 1), &trans(0, 2), &trans(0, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(1, 0), &trans(1, 1), &trans(1, 2), &trans(1, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(2, 0), &trans(2, 1), &trans(2, 2), &trans(2, 3));
					fgets(buffer, 1024, f);
					sscanf(buffer, "%lf %lf %lf %lf", &trans(3, 0), &trans(3, 1), &trans(3, 2), &trans(3, 3));

					data_.push_back(FramedTransformation(frame1, frame2, trans));
				}
			}
			fclose(f);
		}
	}
};

struct QuadPose{
	std::vector<std::vector<double>> quad;		//double tx, ty, tz, qx, qy, qz, qw;  first line number
	std::vector<int> id;
	void LoadFromFile(std::string filename, bool ORB=false)
	{
		std::vector<double> qua;
		qua.resize(7);
		int idx;
		id.clear();
		std::ifstream input(filename);
		while (input.good())
		{
			std::string line;
			std::getline(input, line);
			if (line.empty()) continue;
			std::stringstream ss(line);
			std::string str;
			ss >> str;
			if (str == "#") continue;
			else
			{
				if (!ORB)
				{
					str = str.substr(2);
					sscanf(str.c_str(), "%06d", &idx);
				}
				else
				{
					float f = atof(str.c_str());
					idx = static_cast<int>(f);
				}
			}
			ss >> qua[0] >> qua[1] >> qua[2] >> qua[3] >> qua[4] >> qua[5] >> qua[6];
			quad.push_back(qua);
			id.push_back(idx);
		}
		input.close();
	}
};

class Convert
{
public:
	Convert(){};
	~Convert(){};

	Eigen::Matrix4d QuadToMatrix4d(const std::vector<double>& vec7d) {		//vec7d should be tx, ty, tz, qx, qy, qz, qw
		Eigen::Matrix4d m = Eigen::Matrix4d::Identity();

		Eigen::Quaterniond quad;
		quad.x() = vec7d[3];
		quad.y() = vec7d[4];
		quad.z() = vec7d[5];
		quad.w() = vec7d[6];
		Eigen::Matrix3d mat3 = quad.toRotationMatrix();

		m.block(0, 0, 3, 3) = mat3;
		m(0, 3) = vec7d[0];
		m(1, 3) = vec7d[1];
		m(2, 3) = vec7d[2];

		return m;
	}

	std::vector<double> Matrix4dToQuad(const Eigen::Matrix4d& m){
		Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
		r << m(0, 0), m(0, 1), m(0, 2),
			m(1, 0), m(1, 1), m(1, 2),
			m(2, 0), m(2, 1), m(2, 2);

		Eigen::Quaterniond q(r);

		std::vector<double> result(7, 0.0);
		result[0] = m(0, 3);
		result[1] = m(1, 3);
		result[2] = m(2, 3);
		result[3] = q.x();
		result[4] = q.y();
		result[5] = q.z();
		result[6] = q.w();

		return result;
	}
};