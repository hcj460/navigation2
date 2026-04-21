/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.h 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#ifndef NAV2_AMCL__MAP__MAP_HPP_
#define NAV2_AMCL__MAP__MAP_HPP_

#include <stdint.h>

//C / C++ 兼容处理
#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations  前向声明
struct _rtk_fig_t;


// Limits  WiFi 最大层数
#define MAP_WIFI_MAX_LEVELS 8

//内存对齐（非常重要） 强制结构体按 1 字节对齐
//否则二进制读写、内存映射、ROS2 消息会出错。
// make sure that the sizeof(map_cell_t) == 5
#pragma pack(push, 1)
//地图栅格单元 map_cell_t
// Description for a single map cell.
typedef struct
{
  //表示占据状态 -1 Free（空闲）,0 Unknown（未知），1 占据
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int8_t occ_state;

  //到最近 占据栅格​ 的距离
  // Distance to the nearest occupied cell
  float occ_dist;

  //WiFi 数据字段（已注释） 若启用，每个 cell 存 8 个 RSSI 值
  // Wifi levels
  // int wifi_levels[MAP_WIFI_MAX_LEVELS];
} map_cell_t;
#pragma pack(pop)

//完整地图结构 map_t
// Description for a map
typedef struct
{
  //地图原点；地图是概念性较大地图上的视口。
  //地图左下角在世界坐标系中的位置
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;

  //分辨率
  // Map scale (m/cell)
  double scale;

  //地图尺寸
  //size_x：宽度（列数） size_y：高度（行数）
  // Map dimensions (number of cells)
  int size_x, size_y;

  // The map data, stored as a grid 栅格数据存储
  map_cell_t * cells;

  //构建 CSpace（配置空间）​ 的最大距离
  //超过此距离的障碍物忽略
  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;
} map_t;


/**************************************************************************
 * Basic map functions
 **************************************************************************/

// Create a new (empty) map 创建地图
map_t * map_alloc(void);

// Destroy a map 创建地图
void map_free(map_t * map);

// Update the cspace distances 更新 CSpace
//计算每个 cell 的 occ_dist，使用 BFS / 距离变换算法
void map_update_cspace(map_t * map, double max_occ_dist);


/**************************************************************************
 * Range functions
 **************************************************************************/

// Extract a single range reading from the map 测距函数
/*
从 (ox, oy)出发
沿角度 oa
在 max_range内
返回第一个障碍物的距离
*/
double map_calc_range(map_t * map, double ox, double oy, double oa, double max_range);


/**************************************************************************
 * GUI/diagnostic functions
 **************************************************************************/

// Draw the occupancy grid 绘制占据栅格图
void map_draw_occ(map_t * map, struct _rtk_fig_t * fig);

// Draw the cspace map 绘制距离场（CSpace）
void map_draw_cspace(map_t * map, struct _rtk_fig_t * fig);

// Draw a wifi map 绘制 WiFi 信号分布（未启用）
void map_draw_wifi(map_t * map, struct _rtk_fig_t * fig, int index);


/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// Convert from map index to world coords
//地图索引 → 世界坐标
//(i, j)是地图中心为原点的索引
#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)

// Convert from world coords to map coords
//世界坐标 → 地图索引
//四舍五入取整
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

// Test to see if the given map coords lie within the absolute map bounds.
//边界检查
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.
//线性索引计算
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

#ifdef __cplusplus
}
#endif

#endif  // NAV2_AMCL__MAP__MAP_HPP_
