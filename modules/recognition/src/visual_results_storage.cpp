/*
 * visual_results_storage.cpp
 *
 *  Created on: Oct 30, 2015
 *      Author: Matin Velas
 */

#include <v4r/core/macros.h>
#include <v4r/recognition/visual_results_storage.h>

namespace v4r {

int VisualResultsStorage::view_counter = -1;

V4R_EXPORTS
std::string VisualResultsStorage::getNameOf(const std::string what, const std::string ext) const {
	std::stringstream view_no;
	view_no << std::setfill('0') << std::setw(2) << view_counter;
	return base_dir + "/" + view_no.str() + "_" + what + ext;
}

}
