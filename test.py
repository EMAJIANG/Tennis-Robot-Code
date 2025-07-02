from isaacsim import SimulationApp
# 显式启用 ROS2 bridge 扩展
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting"
    # "open_usd": None,
    # "exts": [
    #     "omni.isaac.ros2_bridge",
    #     "omni.isaac.core",  # 确保其他依赖扩展也加载
    # ]
})

# import logging
# import omni.graph.core as og
# import omni.kit.app
# import omni.usd
# logger = logging.getLogger(__name__)
 
# def create_with_omnigraph():
#     prim_path = "/World/XXX"
#     joint_states_topic = "/XXX/joint_states"
#     joint_command_topic = "/XXX/joint_commands"
#     stage = omni.usd.get_context().get_stage()
#     stage.DefinePrim(prim_path)
#     prim = stage.GetPrimAtPath(prim_path)
#     prim.GetReferences().AddReference("XXX.usd")

#     keys = og.Controller.Keys

#     (graph_handle, list_of_nodes, _, _) = og.Controller.edit(

#         {"graph_path": "/action_graph", "evaluator_name": "execution"},

#         {

#             keys.CREATE_NODES: [

#                 ("tick", "omni.graph.action.OnTick"),

#                 ("print","omni.graph.ui_nodes.PrintText")

#             ],

#             keys.SET_VALUES: [

#                 ("print.inputs:text", "Hello World"),

#                 ("print.inputs:logLevel","Warning")                 # setting the log level to warning so we can see the printout in terminal

#             ],

#             keys.CONNECT: [

#                 ("tick.outputs:tick", "print.inputs:execIn")

#             ],

#         },

#     )
#     # og.Controller.edit(
#     #     {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
#     #     {
#     #     og.Controller.Keys.CREATE_NODES: [
#     #         ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#     #         ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
#     #         ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
#     #         ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
#     #         ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
#     #         ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
#     #         ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
#     #         ("ConstantTokenTarget", "omni.graph.nodes.ConstantToken"),
#     #         ("ToTarget", "omni.graph.nodes.ToTarget"),
#     #         ("ConstantStringJointStatesTopic", "omni.graph.nodes.ConstantString"),
#     #         ("ConstantStringJointCommandTopic", "omni.graph.nodes.ConstantString"),
#     #         ],
#     #     og.Controller.Keys.CONNECT: [
#     #         ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
#     #         ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
#     #         ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
#     #         ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
#     #         ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
#     #         ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
#     #         ("Context.outputs:context", "PublishJointState.inputs:context"),
#     #         ("Context.outputs:context", "SubscribeJointState.inputs:context"),
#     #         ("Context.outputs:context", "PublishClock.inputs:context"),
#     #         ("ConstantTokenTarget.inputs:value", "ToTarget.inputs:value"),
#     #         ("ToTarget.outputs:converted", "PublishJointState.inputs:targetPrim"),
#     #         ("ToTarget.outputs:converted", "ArticulationController.inputs:targetPrim"),
#     #         ("ConstantStringJointStatesTopic.inputs:value", "PublishJointState.inputs:topicName"),
#     #         ("ConstantStringJointCommandTopic.inputs:value", "SubscribeJointState.inputs:topicName"),
#     #         ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
#     #         ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
#     #         ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
#     #         ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
#     #         ],
#     #     og.Controller.Keys.SET_VALUES: [
#     #         ("ConstantTokenTarget.inputs:value", prim_path),
#     #         ("ConstantStringJointStatesTopic.inputs:value", joint_states_topic),
#     #         ("ConstantStringJointCommandTopic.inputs:value", joint_command_topic),
#     #         ],
#     #     },
#     # )
 
# if __name__ == '__main__':
#     manager = omni.kit.app.get_app().get_extension_manager()
#     manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
#     print("ros2_bridge enabled: " + str(manager.is_extension_enabled("omni.isaac.ros2_bridge")))
#     logger.info("ros2_bridge enabled: " + str(manager.is_extension_enabled("omni.isaac.ros2_bridge")))
#     create_with_omnigraph()
#     og_usd_path = 'usd/XXX_ros.usd'
#     omni.usd.get_context().save_as_stage(og_usd_path)
#     print("Done")
#     logger.info("Done")
#     # simulation_app.close()