// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <yaml-cpp/yaml.h>

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <string>


using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

int main(int argc, char *argv[])
{
	std::string str_ref = "/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/white_20190611/top_tail/raw/ref.pcd";
	std::string str_data = "/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/white_20190611/top_tail/raw/data.pcd";
	std::string str_ref_save = "/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/white_20190611/top_tail/raw/ref_filter.pcd";
	std::string str_data_save = "/home/jjiao/catkin_ws/src/lidar_appearance_calibration/data/white_20190611/top_tail/raw/data_filter.pcd";

	// merge pointcloud
	DP ref = DP::load(str_ref);
	DP data = DP::load(str_data);

	std::shared_ptr<PM::DataPointsFilter> subsample =
		PM::get().DataPointsFilterRegistrar.create(
			"BoundingBoxDataPointsFilter", PM::Parameters({
			{
				{"xMin", "-15.0"},
				{"xMax", "12.0"},
				{"yMin", "-13"},
				{"yMax", "15"},
				{"zMin", "-inf"},
				{"zMax", "inf"},
				{"removeInside", "0"}
			}
			}));
	ref = subsample->filter(ref);

	subsample =
		PM::get().DataPointsFilterRegistrar.create(
			"BoundingBoxDataPointsFilter", PM::Parameters({
			{
				{"xMin", "5.0"},
				{"xMax", "15.0"},
				{"yMin", "0"},
				{"yMax", "15"},
				{"zMin", "-2.0"},
				{"zMax", "inf"},
				{"removeInside", "1"}
			}
			}));
	ref = subsample->filter(ref);

// 	subsample =
// 		PM::get().DataPointsFilterRegistrar.create(
// 			"BoundingBoxDataPointsFilter", PM::Parameters({
// 			{
// 				{"xMin", "3.5"},
// 				{"xMax", "5"},
// 				{"yMin", "5.5"},
// 				{"yMax", "13"},
// 				{"zMin", "-2"},
// 				{"zMax", "inf"},
// 				{"removeInside", "1"}
// 			}
// 			}));
// 	ref = subsample->filter(ref);	
	
	subsample =
		PM::get().DataPointsFilterRegistrar.create(
			"BoundingBoxDataPointsFilter", PM::Parameters({
			{
				{"xMin", "0.5"},
				{"xMax", "13"},
				{"yMin", "-10"},
				{"yMax", "10"},
				{"zMin", "-inf"},
				{"zMax", "inf"},
				{"removeInside", "0"}
			}
			}));
	data = subsample->filter(data);
	
// 	subsample =
// 		PM::get().DataPointsFilterRegistrar.create(
// 			"BoundingBoxDataPointsFilter", PM::Parameters({
// 			{
// 				{"xMin", "0.0"},
// 				{"xMax", "2"},
// 				{"yMin", "-2"},
// 				{"yMax", "3"},
// 				{"zMin", "-inf"},
// 				{"zMax", "inf"},
// 				{"removeInside", "1"}
// 			}
// 			}));
// 	data = subsample->filter(data);	

	ref.save(str_ref_save);
	data.save(str_data_save);

	return 0;
}
