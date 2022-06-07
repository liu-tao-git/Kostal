Project/mainpathok-MainPlan.plan                                                                    000644  000000  000000  00000060152 14221220060 015327  0                                                                                                    ustar 00                                                                000000  000000                                                                                                                                                                         plan_name: "mainpathok-MainPlan"
config {
  node_name: "rootNode"
  pt_name: "Plan"
  pt_type: "PLAN"
  switch_tcp_param {
    switch_tcp: false
    tool_name: ""
    tcp_index: 0
  }
}
transit_list {
  start_node_name: "MoveL1"
  end_node_name: "MoveL11"
  transit_period: 0
  trigger_condition {
    condition_type: "OR"
    trigger_condition {
      condition_type: "EQUAL"
      lhs_param {
        name: "reachedTarget"
        module_name: "MoveL1"
        type: "BOOL"
        category: "PT_STATE"
        robotUnit: "none"
      }
      rhs_param {
        type: "BOOL"
        category: "CONST"
        data: "1"
      }
    }
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "CartesianMomentZ"
        module_name: "SYSTEM"
        type: "DOUBLE"
        category: "SYS_STATE"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "1.5"
      }
    }
  }
}
transit_list {
  start_node_name: "MoveL12"
  end_node_name: "MoveL111"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "MoveL12"
      type: "BOOL"
      category: "PT_STATE"
      robotUnit: "none"
    }
    rhs_param {
      type: "BOOL"
      category: "CONST"
      data: "1"
    }
  }
}
transit_list {
  start_node_name: "CaliForceSensor0"
  end_node_name: "PoseTCP1"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "terminated"
      module_name: "CaliForceSensor0"
      type: "BOOL"
      category: "PT_STATE"
      robotUnit: "none"
    }
    rhs_param {
      type: "BOOL"
      category: "CONST"
      data: "1"
    }
  }
}
transit_list {
  start_node_name: "MoveL111"
  end_node_name: "MoveL12"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "MoveL111"
      type: "BOOL"
      category: "PT_STATE"
      robotUnit: "none"
    }
    rhs_param {
      type: "BOOL"
      category: "CONST"
      data: "1"
    }
  }
}
transit_list {
  start_node_name: "Hold0"
  end_node_name: "MoveL112"
  transit_period: 0
  trigger_condition {
    condition_type: "GREATER"
    lhs_param {
      name: "timePeriod"
      module_name: "Hold0"
      type: "DOUBLE"
      category: "PT_STATE"
      robotUnit: "s"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "1"
    }
  }
  param_assignment {
    lhs_param {
      name: "stdPose"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    rhs_param {
      name: "tcpPose"
      module_name: "SYSTEM"
      type: "COORD"
      category: "SYS_STATE"
    }
  }
}
transit_list {
  start_node_name: "MoveL11"
  end_node_name: "Hold2"
  transit_period: 0
  trigger_condition {
    condition_type: "OR"
    trigger_condition {
      condition_type: "EQUAL"
      lhs_param {
        name: "reachedTarget"
        module_name: "MoveL11"
        type: "BOOL"
        category: "PT_STATE"
        robotUnit: "none"
      }
      rhs_param {
        type: "BOOL"
        category: "CONST"
        data: "1"
      }
    }
    trigger_condition {
      condition_type: "LESS"
      lhs_param {
        name: "CartesianMomentZ"
        module_name: "SYSTEM"
        type: "DOUBLE"
        category: "SYS_STATE"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "-1.5"
      }
    }
  }
}
transit_list {
  start_node_name: "Hold2"
  end_node_name: "End0"
  transit_period: 0
  trigger_condition {
    condition_type: "GREATER"
    lhs_param {
      name: "timePeriod"
      module_name: "Hold2"
      type: "DOUBLE"
      category: "PT_STATE"
      robotUnit: "s"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "5"
    }
  }
}
transit_list {
  start_node_name: "MoveL112"
  end_node_name: "MoveL1"
  transit_period: 0
  trigger_condition {
    condition_type: "OR"
    trigger_condition {
      condition_type: "EQUAL"
      lhs_param {
        name: "reachedTarget"
        module_name: "MoveL112"
        type: "BOOL"
        category: "PT_STATE"
        robotUnit: "none"
      }
      rhs_param {
        type: "BOOL"
        category: "CONST"
        data: "1"
      }
    }
    trigger_condition {
      condition_type: "LESS"
      lhs_param {
        name: "CartesianMomentZ"
        module_name: "SYSTEM"
        type: "DOUBLE"
        category: "SYS_STATE"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "-1.5"
      }
    }
  }
}
transit_list {
  start_node_name: "PoseTCP1"
  end_node_name: "Hold0"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "PoseTCP1"
      type: "BOOL"
      category: "PT_STATE"
      robotUnit: "none"
    }
    rhs_param {
      type: "BOOL"
      category: "CONST"
      data: "1"
    }
  }
}
transit_list {
  start_node_name: "startNode"
  end_node_name: "MoveL6"
  transit_period: 0
  trigger_condition {
    condition_type: "NO_CHECK"
  }
}
transit_list {
  start_node_name: "MoveL6"
  end_node_name: "CaliForceSensor0"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "MoveL6"
      type: "BOOL"
      category: "PT_STATE"
      robotUnit: "none"
    }
    rhs_param {
      type: "BOOL"
      category: "CONST"
      data: "1"
    }
  }
}
node_list {
  node_name: "CaliForceSensor0"
  pt_name: "CaliForceSensor"
  pt_type: "CALIFORCESENSOR"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 1
  }
}
node_list {
  node_name: "PoseTCP1"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 1
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "PoseTCP1"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0.155866512"
      data: "-0.471328993"
      data: "0.329937531"
      data: "-160.119494"
      data: "-36.6072087"
      data: "127.108242"
      data: "WORLD"
      data: "WORLD_ORIGIN"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "PoseTCP1"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "-0.1"
      data: "0"
      data: "0"
      data: "0"
      data: "TRAJ"
      data: "GOAL"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "-45"
      data: "TRAJ"
      data: "GOAL"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "PoseTCP1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.05"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "PoseTCP1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "30"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "PoseTCP1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "1.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotAcc"
      module_name: "PoseTCP1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "blendRadiusLevel"
      module_name: "PoseTCP1"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "blendRadiusLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "PoseTCP1"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "INT"
      category: "CONST"
      data: "1"
    }
  }
  param_assignment {
    lhs_param {
      name: "preferJntPos"
      module_name: "PoseTCP1"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "jointPos"
      module_name: "SYSTEM"
      type: "VEC_7d"
      category: "SYS_STATE"
    }
  }
}
node_list {
  node_name: "Hold0"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 2
  }
}
node_list {
  node_name: "MoveL1"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveL1"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "-50"
      data: "VAR"
      data: "stdPose"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveL1"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "MoveL1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.01"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveL1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "15"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "MoveL1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "1.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotAcc"
      module_name: "MoveL1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "blendRadiusLevel"
      module_name: "MoveL1"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "blendRadiusLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "MoveL1"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "targetTolLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "preferJntPos"
      module_name: "MoveL1"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "jointPos"
      module_name: "SYSTEM"
      type: "VEC_7d"
      category: "SYS_STATE"
    }
  }
}
node_list {
  node_name: "End0"
  pt_name: "End"
  pt_type: "END"
  switch_tcp_param {
    switch_tcp: false
    tool_name: ""
    tcp_index: 0
  }
}
node_list {
  node_name: "Hold1"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 0
  }
}
node_list {
  node_name: "MoveL11"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveL11"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "50"
      data: "VAR"
      data: "stdPose"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveL11"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "MoveL11"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.01"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveL11"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "15"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "MoveL11"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "1.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotAcc"
      module_name: "MoveL11"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "blendRadiusLevel"
      module_name: "MoveL11"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "blendRadiusLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "MoveL11"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "targetTolLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "preferJntPos"
      module_name: "MoveL11"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "jointPos"
      module_name: "SYSTEM"
      type: "VEC_7d"
      category: "SYS_STATE"
    }
  }
}
node_list {
  node_name: "MoveL12"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveL12"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "-30"
      data: "VAR"
      data: "stdPose"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveL12"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "MoveL12"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.01"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveL12"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "15"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "MoveL12"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "1.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotAcc"
      module_name: "MoveL12"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "blendRadiusLevel"
      module_name: "MoveL12"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "blendRadiusLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "MoveL12"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "targetTolLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "preferJntPos"
      module_name: "MoveL12"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "jointPos"
      module_name: "SYSTEM"
      type: "VEC_7d"
      category: "SYS_STATE"
    }
  }
}
node_list {
  node_name: "MoveL111"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveL111"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "5"
      data: "VAR"
      data: "stdPose"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveL111"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "MoveL111"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.01"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveL111"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "15"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "MoveL111"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "1.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotAcc"
      module_name: "MoveL111"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "blendRadiusLevel"
      module_name: "MoveL111"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "blendRadiusLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "MoveL111"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "targetTolLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "preferJntPos"
      module_name: "MoveL111"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "jointPos"
      module_name: "SYSTEM"
      type: "VEC_7d"
      category: "SYS_STATE"
    }
  }
}
node_list {
  node_name: "MoveL112"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveL112"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "50"
      data: "VAR"
      data: "stdPose"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveL112"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "MoveL112"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.01"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveL112"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "15"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "MoveL112"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "1.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotAcc"
      module_name: "MoveL112"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "blendRadiusLevel"
      module_name: "MoveL112"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "blendRadiusLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "MoveL112"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "targetTolLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "preferJntPos"
      module_name: "MoveL112"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "jointPos"
      module_name: "SYSTEM"
      type: "VEC_7d"
      category: "SYS_STATE"
    }
  }
}
node_list {
  node_name: "Hold2"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: false
    tool_name: ""
    tcp_index: 0
  }
}
node_list {
  node_name: "MoveL6"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest2"
    tcp_index: 0
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveL6"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "-0.15"
      data: "0"
      data: "0"
      data: "0"
      data: "TRAJ"
      data: "START"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveL6"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "MoveL6"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.05"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveL6"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotVel"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "MoveL6"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "1.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotAcc"
      module_name: "MoveL6"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxRotAcc"
      type: "DOUBLE"
      category: "CONST"
      data: "300"
    }
  }
  param_assignment {
    lhs_param {
      name: "blendRadiusLevel"
      module_name: "MoveL6"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "blendRadiusLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "MoveL6"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "targetTolLevel"
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
  param_assignment {
    lhs_param {
      name: "preferJntPos"
      module_name: "MoveL6"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "preferJntPos"
      type: "VEC_7d"
      category: "CONST"
      data: "0"
      data: "-40"
      data: "0"
      data: "90"
      data: "0"
      data: "40"
      data: "0"
    }
  }
}
tool_names: "Tooltest2"
tool_names: "Tooltest"
var_list {
  name: "stdPose"
  module_name: "rootNode"
  type: "COORD"
  category: "PLAN_VAR"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "WORLD"
  data: "WORLD_ORIGIN"
  robotUnit: "m-deg"
}
plan_log {
  enable_log: true
  time_interval: 1
  max_duration: 30
}
plan_desc: ""
                                                                                                                                                                                                                                                                                                                                                                                                                      Project/mainpathok.proj                                                                             000644  000000  000000  00000002222 14221220060 013644  0                                                                                                    ustar 00                                                                000000  000000                                                                                                                                                                         project_name: "mainpathok"
version: "v2.8-develop"
updated_date: "2022-03-31_11-29-52"
plan_file_name: "mainpathok-MainPlan.plan"
scene_width: 3466
scene_height: 2260
plan_config {
  plan_name: "mainpathok-MainPlan"
  parent_node_name: ""
  node_config {
    node_name: "rootNode"
    x: 0
    y: 0
  }
  block_list {
    node_name: "startNode"
    x: 77
    y: 385
  }
  block_list {
    node_name: "CaliForceSensor0"
    x: 616
    y: 385
  }
  block_list {
    node_name: "PoseTCP1"
    x: 616
    y: 539
  }
  block_list {
    node_name: "Hold0"
    x: 385
    y: 693
  }
  block_list {
    node_name: "MoveL1"
    x: 924
    y: 693
  }
  block_list {
    node_name: "End0"
    x: 1694
    y: 693
  }
  block_list {
    node_name: "Hold1"
    x: 847
    y: 308
  }
  block_list {
    node_name: "MoveL11"
    x: 1232
    y: 693
  }
  block_list {
    node_name: "MoveL12"
    x: 1001
    y: 924
  }
  block_list {
    node_name: "MoveL111"
    x: 1309
    y: 924
  }
  block_list {
    node_name: "MoveL112"
    x: 616
    y: 693
  }
  block_list {
    node_name: "Hold2"
    x: 1463
    y: 693
  }
  block_list {
    node_name: "MoveL6"
    x: 308
    y: 385
  }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              