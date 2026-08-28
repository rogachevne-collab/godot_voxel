#ifndef VOXEL_LOD_TERRAIN_UPDATE_TASK_H
#define VOXEL_LOD_TERRAIN_UPDATE_TASK_H

#include "../../engine/ids.h"
#include "../../engine/priority_dependency.h"
#include "../../storage/voxel_data.h"
#include "../../util/containers/std_vector.h"
#include "../../util/tasks/threaded_task.h"
#include "voxel_lod_terrain_update_data.h"

namespace zylann::voxel {

struct StreamingDependency;
struct MeshingDependency;
class BufferedTaskScheduler;

// Runs a part of the update loop of a VoxelLodTerrain.
// This part can run on another thread, so multiple terrains can update in parallel.
// There must be only one running at once per terrain.
// Note, this task does not include meshing and voxel generation. These are done with different tasks.
//
// IMPORTANT: The work done by this task must not involve any call to Godot's servers, directly or indirectly.
// These are deferred to the main thread.
//
class VoxelLodTerrainUpdateTask : public IThreadedTask {
public:
	VoxelLodTerrainUpdateTask( 
			std::shared_ptr<VoxelData> p_data, 
			std::shared_ptr<VoxelLodTerrainUpdateData> p_update_data, 
			std::shared_ptr<StreamingDependency> p_streaming_dependency, 
			std::shared_ptr<MeshingDependency> p_meshing_dependency, 
			std::shared_ptr<PriorityDependency::ViewersData> p_shared_viewers_data, 
			const Vector3 p_viewer_pos, 
			const VolumeID p_volume_id, 
			const Transform3D p_volume_transform 
			) :
			_data(p_data),
			_update_data(p_update_data),
			_streaming_dependency(p_streaming_dependency),
			_meshing_dependency(p_meshing_dependency),
			_shared_viewers_data(p_shared_viewers_data),
			_viewer_pos(p_viewer_pos),
			_volume_id(p_volume_id),
			_volume_transform(p_volume_transform) {}

	const char *get_debug_name() const override {
		return "VoxelLodTerrainUpdate";
	}

	void run(ThreadedTaskContext &ctx) override;

	// Functions also used outside of this task

	static void flush_pending_lod_edits(
			VoxelLodTerrainUpdateData::State &state,
			VoxelData &data,
			const int mesh_block_size
	);

	static uint8_t get_transition_mask(
			const VoxelLodTerrainUpdateData::State &state,
			const Vector3i block_pos,
			unsigned int lod_index,
			unsigned int lod_count
	);

	// To use on loaded blocks
	static inline void schedule_mesh_update(
			VoxelLodTerrainUpdateData::MeshBlockState &block,
			const Vector3i bpos,
			StdVector<VoxelLodTerrainUpdateData::MeshToUpdate> &blocks_pending_update,
			const bool require_visual,
			const bool collision_changed = false
	) {
		if (block.state == VoxelLodTerrainUpdateData::MESH_UPDATE_NOT_SENT) {
			// Already queued and not sent yet. The request reads the block's current state when it goes out,
			// so it will carry this change too.
			if (collision_changed) {
				++block.collision_revision;
			}
			return;
		}
		// Parking hands the block to the visibility system, and that system only ever reaches blocks whose
		// parent LOD is active. A parent becomes active when a mesh result arrives for it — so a block that
		// has never produced one has no way back: it needs a mesh to be reached, and it needs to be reached
		// to be given a mesh. Left parked, it sits in `MESH_NEED_UPDATE` for the rest of the session with no
		// mesh, no error and no second opinion. Host games that call `post_edit_modifiers` while streaming is
		// still filling the area (bm-underworld does it per pasted chunk, all through world opening) walk into
		// this on every boot: the whole terrain stays invisible.
		const bool never_meshed = !block.visual_loaded && !block.collision_loaded;
		if (block.visual_active || block.collision_active || never_meshed) {
			// Schedule an update
			if (collision_changed) {
				++block.collision_revision;
			}
			block.state = VoxelLodTerrainUpdateData::MESH_UPDATE_NOT_SENT;
			block.update_list_index = blocks_pending_update.size();
			blocks_pending_update.push_back(
					VoxelLodTerrainUpdateData::MeshToUpdate{ bpos, TaskCancellationToken(), require_visual }
			);
		} else {
			// Just mark it as needing update, so the visibility system will schedule its update when needed.
			// `set_active` in the clipbox streamer is the other half of that promise.
			//
			// No collision revision bump here, deliberately. The revision exists to reject a collider built
			// before a change, and it can only do that job while every bump is paired with a request that
			// carries the new number. Parking sends no request, so a bump here just runs the counter ahead of
			// the results and starves the block: a host game remeshing generated areas while streaming fills
			// them (bm-underworld does it per pasted chunk) has every arriving collider rejected as stale and
			// gets no terrain collision at all — and so no navigation mesh — until an ordinary edit happens to
			// land in a quiet moment. The block is rebuilt when `set_active` un-parks it, and that request
			// reads the revision as it stands.
			block.state = VoxelLodTerrainUpdateData::MESH_NEED_UPDATE;
		}
	}

	static void send_block_save_requests(
			const VolumeID volume_id,
			const Span<VoxelData::BlockToSave> blocks_to_save,
			const std::shared_ptr<StreamingDependency> &stream_dependency,
			BufferedTaskScheduler &task_scheduler,
			const std::shared_ptr<AsyncDependencyTracker> tracker,
			const bool with_flush
	);

private:
	std::shared_ptr<VoxelData> _data;
	std::shared_ptr<VoxelLodTerrainUpdateData> _update_data;
	std::shared_ptr<StreamingDependency> _streaming_dependency;
	std::shared_ptr<MeshingDependency> _meshing_dependency;
	std::shared_ptr<PriorityDependency::ViewersData> _shared_viewers_data;
	Vector3 _viewer_pos;
	VolumeID _volume_id;
	Transform3D _volume_transform;
};

void update_transition_masks(
		VoxelLodTerrainUpdateData::State &state,
		const uint32_t lods_to_update_transitions,
		const unsigned int lod_count,
		const bool use_refcounts
);

void add_unloaded_saving_blocks(VoxelLodTerrainUpdateData::Lod &lod, Span<const VoxelData::BlockToSave> src);

} // namespace zylann::voxel

#endif // VOXEL_LOD_TERRAIN_UPDATE_TASK_H
