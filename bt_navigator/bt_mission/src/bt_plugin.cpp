// 이 패키지의 BT 노드(DoWork, ScanWhileMoving, ForEachWaypoint, ObstacleMode, StopAndGo)를 플러그인 .so 로 내보낸다.
// BtActionServer 는 BT 노드를 plugin_lib_names 의 라이브러리 이름으로만 등록하기 때문이다
// (task_server_lifecycle 의 기본 plugin_lib_names 에 "bt_mission_plugin" 이 들어 있다).
// task_server 는 이 파일 없이 registerNodes() 를 코드에서 바로 부른다.

#include "behaviortree_cpp_v3/bt_factory.h"
#include "bt_mission/mission_nodes.hpp"

BT_REGISTER_NODES(factory)
{
  bt_mission::registerNodes(factory);
}
