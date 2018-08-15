#ifndef _MYSLAM_SIM_RESOURCES_H_
#define _MYSLAM_SIM_RESOURCES_H_

#include <string>
#include <visualization_msgs/Marker.h>
using namespace std;

namespace MYSLAM {
	class ObjectRviz {

		public:
			ObjectRviz(std::string);
			visualization_msgs::Marker marker;
			map<string,int> m;
	};

	ObjectRviz::ObjectRviz (std::string s) {

		m ["pi2"] = 0;
		m ["bar"] = 1;
		m ["blu"] = 2;
		m ["boo"] = 3;
		m ["box"] = 4;
		m ["cab"] = 5;
		m ["caf"] = 6;
		m ["con"] = 7;
		m ["gre"] = 8;
		m ["hbl"] = 9;
		m ["hyd"] = 10;
		m ["red"] = 11;
		m ["tab"] = 12;
		m ["wro"] = 13;
		m ["doo"] = 14;
		m ["lad"] = 15;
		m ["ybl"] = 16;
		m ["dum"] = 17;
		m ["spe"] = 18;

		int type = m [s];

		marker.action = visualization_msgs::Marker::ADD;
		marker.color.a = 1.0;

		switch (type) {
			case 0: {
				    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
				    marker.mesh_resource = "package://myslam_sim_resources/models/pioneer2dx/meshes/chassis.dae";
				    marker.mesh_use_embedded_materials = true;
				    marker.scale.x = 2; 
				    marker.scale.y = 2;
				    marker.scale.z = 2;
				    marker.color.r = 1.0;
				    marker.color.g = 1.0;
				    marker.color.b = 1.0;
				    break;
				    }
			case 1: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/construction_barrel/meshes/construction_barrel.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.025;
					    marker.scale.y = 0.025;
					    marker.scale.z = 0.025;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 2: {
					    marker.type = visualization_msgs::Marker::CYLINDER;
					    marker.scale.x = 0.8;
					    marker.scale.y = 0.8;
					    marker.scale.z = 2.5;
					    marker.color.r = 0.0;
					    marker.color.g = 0.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 3: {
					    marker.type = visualization_msgs::Marker::CUBE;
					    marker.scale.x = 1;
					    marker.scale.y = 1;
					    marker.scale.z = 2;
					    marker.color.r = 1.0;
					    marker.color.g = 0.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 4: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/cardboard_box/meshes/cardboard_box.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.002;
					    marker.scale.y = 0.002;
					    marker.scale.z = 0.002;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 5: {
					    marker.type = visualization_msgs::Marker::CUBE;
					    marker.scale.x = 1;
					    marker.scale.y = 1.5;
					    marker.scale.z = 2;
					    marker.color.r = 0.0;
					    marker.color.g = 0.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 6: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/cafe_table/meshes/cafe_table.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.02;
					    marker.scale.y = 0.02;
					    marker.scale.z = 0.02;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 7: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/construction_cone/meshes/construction_cone.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.5;
					    marker.scale.y = 0.5;
					    marker.scale.z = 0.5;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 8: {
					    marker.type = visualization_msgs::Marker::CYLINDER;
					    marker.scale.x = 0.8;
					    marker.scale.y = 0.8;
					    marker.scale.z = 2.5;
					    marker.color.r = 0.0;
					    marker.color.g = 1.0;
					    marker.color.b = 0.0;
					    break;
				    }
			case 9: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/cinder_block/meshes/cinder_block.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.04;
					    marker.scale.y = 0.04;
					    marker.scale.z = 0.04;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 10: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/fire_hydrant/meshes/fire_hydrant.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.02;
					    marker.scale.y = 0.02;
					    marker.scale.z = 0.02;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 11: {
					    marker.type = visualization_msgs::Marker::CYLINDER;
					    marker.scale.x = 0.8;
					    marker.scale.y = 0.8;
					    marker.scale.z = 2.5;
					    marker.color.r = 1.0;
					    marker.color.g = 0.0;
					    marker.color.b = 0.0;
					    break;
				    }
			case 12: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/table_marble/meshes/table_lightmap.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.2;
					    marker.scale.y = 0.2;
					    marker.scale.z = 0.2;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 13: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/warehouse_robot/meshes/robot.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.001;
					    marker.scale.y = 0.001;
					    marker.scale.z = 0.001;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 14: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/hinged_door/meshes/door.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.2;
					    marker.scale.y = 0.2;
					    marker.scale.z = 0.2;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 15: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/drc_practice_ladder/meshes/ladder.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.8;
					    marker.scale.y = 0.8;
					    marker.scale.z = 0.8;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 16: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/drc_practice_yellow_parking_block/meshes/block.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.02;
					    marker.scale.y = 0.02;
					    marker.scale.z = 0.02;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 17: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/dumpster/meshes/dumpster.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.04;
					    marker.scale.y = 0.04;
					    marker.scale.z = 0.04;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			case 18: {
					    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
					    marker.mesh_resource = "package://myslam_sim_resources/models/speed_limit_sign/meshes/speed_limit_sign.dae"; 
					    marker.mesh_use_embedded_materials = true;
					    marker.scale.x = 0.6;
					    marker.scale.y = 0.6;
					    marker.scale.z = 0.6;
					    marker.color.r = 1.0;
					    marker.color.g = 1.0;
					    marker.color.b = 1.0;
					    break;
				    }
			default: {
					    marker.type = visualization_msgs::Marker::SPHERE;
					    marker.scale.x = 1;
					    marker.scale.y = 1;
					    marker.scale.z = 1;
					    marker.color.r = 1.0;
					    marker.color.g = 0.5;
					    marker.color.b = 1.5;
					    break;
				    }
		}
	}
}

#endif
