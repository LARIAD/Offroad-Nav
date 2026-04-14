
#ifndef MBF_GRIDMAP_CORE__GRID_MAP_HANDLER_H_
#define MBF_GRIDMAP_CORE__GRID_MAP_HANDLER_H_

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <functional>
#include <grid_map_core/GridMap.hpp>

namespace mbf_gridmap_core {
/**
 * A class dealing with concurrent read/write access to a grid_map using a shared_mutex.
 *
 * The shared_mutex allows for multiple concurrent read access while preventing concurrent access when writing to the
 * GridMap.
 */
class GridmapHandler {
 public:
  GridmapHandler() = default;

  ~GridmapHandler() = default;

  /**
   * @brief Execute a std::function with a read only grid_map::GridMap& as argument. Concurrent access are dealt with
   * internally using a shared_lock on the shared_mutex. This allows for multiple concurrent read access.
   *
   * @param func std::function with a const GridMap& obj as argument.
   * @note GridMap obj will be pass as const.
   */
  void callReadingFunc(const std::function<void(const grid_map::GridMap&)>& func) {
    boost::shared_lock<boost::shared_mutex> sh_lock(mutex_);
    func(grid_map_);
  };

  /**
   * @brief Execute a std::function with a grid_map::GridMap& as argument. Concurrent access are dealt with
   * internally using a unique_lock on the shared_mutex. This prevents any another concurrent access.
   *
   * @param func std::function with a GridMap& obj as argument.
   */
  void callWritingFunc(const std::function<void(grid_map::GridMap&)>& func) {
    boost::unique_lock<boost::shared_mutex> sh_lock(mutex_);
    func(grid_map_);
  };

 private:
  //! shared mutex to manage access by calling "unique_lock" or "shared_lock" to, respectively, write or read
  boost::shared_mutex mutex_;

  /**
   * @brief Get a pointer on grid_map to wright
   *
   * @warning BE CAREFUL THE LOCKING SYSTEM NEED TO BE DONE MANUALLY
   */
  grid_map::GridMap* getWriteGridMap() { return &grid_map_; };

  /**
   * @brief Get a const pointer on grid_map to read
   *
   * @warning BE CAREFUL THE LOCKING SYSTEM NEED TO BE DONE MANUALLY
   */
  const grid_map::GridMap* getReadGridMap() const {
    const grid_map::GridMap* output = &grid_map_;
    return output;
  };

  grid_map::GridMap grid_map_;
};
}; /* namespace mbf_gridmap_core */
#endif /* MBF_GRIDMAP_CORE__GRID_MAP_HANDLER_H_ */