#include "landmark.h"

namespace rgbdt {
  using namespace std;
  
  Landmark* LandmarkMap::get(int id) {
    iterator it=find(id);
    if (it==end())
      return 0;
    return it->second;
  }

  void LandmarkMap::put(Landmark* landmark){
  Landmark* element=get(landmark->id);
    if (! element) {
      insert(make_pair(landmark->id, landmark));
      return;
    }
    if (element==landmark)
	return;
    throw std::runtime_error("error, duplicate insertion");
  }
}
