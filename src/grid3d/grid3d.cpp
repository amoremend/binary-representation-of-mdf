#include "Grid3D/grid3d.hpp"
#include <omp.h>
#include <chrono>

void Grid3d::computeManhattanGrid(int mask_bits)
{
	// Matrix initialization
	// Use auxiliary variable for the mask
	std::vector<uint64_t> matrix3d(m_gridSize, UINT64_MAX);

	// Timing: start timer immediately after initializing the matrix
	auto t_start = std::chrono::high_resolution_clock::now();

	// Get occupied indices
	std::vector<std::array<int,3>> occupied_indices;
	double minX, minY, minZ;
	m_octomap->getMetricMin(minX, minY, minZ);
	for (auto it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it) {
		if (m_octomap->isNodeOccupied(*it)) {
			int ix = (int)((it.getX() - minX) / m_resolution);
			int iy = (int)((it.getY() - minY) / m_resolution);
			int iz = (int)((it.getZ() - minZ) / m_resolution);
			if (ix >= 0 && ix < m_gridSizeX && iy >= 0 && iy < m_gridSizeY && iz >= 0 && iz < m_gridSizeZ) {
				occupied_indices.push_back({ix, iy, iz});
			}
		}
	}

	// Mask table
	std::vector<uint64_t> mask_table(mask_bits+1);
	for (int d = 0; d <= mask_bits; ++d) {
		if (d == 0) mask_table[d] = 0ULL;
		else if (d >= 64) mask_table[d] = UINT64_MAX; // evitar shift >= 64 (indefinido)
		else mask_table[d] = (1ULL << d) - 1ULL;
	}

	// Manhattan offsets
	std::vector<std::array<int,4>> offsets;
	for (int dz = -mask_bits; dz <= mask_bits; ++dz) {
		for (int dy = -mask_bits; dy <= mask_bits; ++dy) {
			for (int dx = -mask_bits; dx <= mask_bits; ++dx) {
				int d = std::abs(dx) + std::abs(dy) + std::abs(dz);
				if (d <= mask_bits) {
					offsets.push_back({dx,dy,dz,d});
				}
			}
		}
	}

	// Apply mask (parallelized)
	#pragma omp parallel for schedule(static)
	for (long k = 0; k < (long)occupied_indices.size(); ++k) {
		int ix = occupied_indices[k][0];
		int iy = occupied_indices[k][1];
		int iz = occupied_indices[k][2];
		for (auto& off : offsets) {
			int jx = ix + off[0];
			int jy = iy + off[1];
			int jz = iz + off[2];
			int d  = off[3];
			if (jx < 0 || jx >= m_gridSizeX || jy < 0 || jy >= m_gridSizeY || jz < 0 || jz >= m_gridSizeZ) continue;
			int index = jx + jy*m_gridStepY + jz*m_gridStepZ;
			uint64_t mask = mask_table[d];
			#pragma omp atomic
			matrix3d[index] &= mask;
		}
	}
	// Stop timer before converting the mask to Manhattan distance
	auto t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> elapsed = t_end - t_start;
	ROS_INFO("Grid generation time: %.3f ms (from matrix initialization to before mask-to-distance conversion)", elapsed.count());

	// Convert mask to Manhattan distance and store as float
	#pragma omp parallel for schedule(static)
	for (int i = 0; i < m_gridSize; ++i) {
		uint64_t t = matrix3d[i];
		int cnt = 0;
		while ((t & 1ULL) && cnt < 64) {
			++cnt;
			t >>= 1;
		}
		m_grid[i].man = static_cast<float>(cnt);
	}
	ROS_INFO("Manhattan grid computation finished (distance stored as float in m_grid[].man).");
}
