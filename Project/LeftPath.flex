Project/LeftPath-MainPlan.plan                                                                      000644  000000  000000  00000243426 14250030100 014706  0                                                                                                    ustar 00                                                                000000  000000                                                                                                                                                                         plan_name: "LeftPath-MainPlan"
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
  start_node_name: "CaliForceSensor0"
  end_node_name: "Insert"
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
  start_node_name: "Insert"
  end_node_name: "GetTCP"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "Insert"
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
  param_assignment {
    lhs_param {
      name: "gearIndex"
      module_name: "rootNode"
      type: "INT"
      category: "PLAN_VAR"
      robotUnit: "none"
    }
    rhs_param {
      type: "INT"
      category: "CONST"
      data: "0"
    }
  }
}
transit_list {
  start_node_name: "startNode"
  end_node_name: "Retreat"
  transit_period: 0
  trigger_condition {
    condition_type: "NO_CHECK"
  }
}
transit_list {
  start_node_name: "Retreat"
  end_node_name: "Ready"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "Retreat"
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
  start_node_name: "Ready"
  end_node_name: "CaliForceSensor0"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "Ready"
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
  start_node_name: "MoveToGear"
  end_node_name: "PathMain"
  transit_period: 0
  trigger_condition {
    condition_type: "AND"
    trigger_condition {
      condition_type: "EQUAL"
      lhs_param {
        name: "reachedTarget"
        module_name: "MoveToGear"
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
      condition_type: "EQUAL"
      lhs_param {
        name: "gearIndex"
        module_name: "rootNode"
        type: "INT"
        category: "PLAN_VAR"
        robotUnit: "none"
      }
      rhs_param {
        type: "INT"
        category: "CONST"
        data: "0"
      }
    }
  }
}
transit_list {
  start_node_name: "GetTCP"
  end_node_name: "Calculate"
  transit_period: 0
  trigger_condition {
    condition_type: "GREATER"
    lhs_param {
      name: "timePeriod"
      module_name: "GetTCP"
      type: "DOUBLE"
      category: "PT_STATE"
      robotUnit: "s"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.001"
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
  start_node_name: "Calculate"
  end_node_name: "MoveToGear"
  transit_period: 0
  trigger_condition {
    condition_type: "GREATER"
    lhs_param {
      name: "timePeriod"
      module_name: "Calculate"
      type: "DOUBLE"
      category: "PT_STATE"
      robotUnit: "s"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.001"
    }
  }
  param_operation {
    operator_type: "ELEMENT_GET"
    in_param_a {
      name: "gearPoseOffset"
      module_name: "rootNode"
      type: "ARRAY_COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    in_param_b {
      name: "gearIndex"
      module_name: "rootNode"
      type: "INT"
      category: "PLAN_VAR"
      robotUnit: "none"
    }
    out_param {
      name: "gearPose"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
  }
  param_operation {
    operator_type: "MULTIPLICATION"
    in_param_a {
      name: "stdPose"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    in_param_b {
      name: "gearPose"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    out_param {
      name: "gearPose"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
  }
}
transit_list {
  start_node_name: "MoveToGear"
  end_node_name: "Path1"
  transit_period: 0
  trigger_condition {
    condition_type: "AND"
    trigger_condition {
      condition_type: "EQUAL"
      lhs_param {
        name: "reachedTarget"
        module_name: "MoveToGear"
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
      condition_type: "EQUAL"
      lhs_param {
        name: "gearIndex"
        module_name: "rootNode"
        type: "INT"
        category: "PLAN_VAR"
        robotUnit: "none"
      }
      rhs_param {
        type: "INT"
        category: "CONST"
        data: "1"
      }
    }
  }
}
transit_list {
  start_node_name: "PathMain"
  end_node_name: "MoveToGear1"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "terminated"
      module_name: "PathMain"
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
  param_assignment {
    lhs_param {
      name: "gearIndex"
      module_name: "rootNode"
      type: "INT"
      category: "PLAN_VAR"
      robotUnit: "none"
    }
    rhs_param {
      type: "INT"
      category: "CONST"
      data: "1"
    }
  }
}
transit_list {
  start_node_name: "Path1"
  end_node_name: "MoveToGear1"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "terminated"
      module_name: "Path1"
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
  param_assignment {
    lhs_param {
      name: "gearIndex"
      module_name: "rootNode"
      type: "INT"
      category: "PLAN_VAR"
      robotUnit: "none"
    }
    rhs_param {
      type: "INT"
      category: "CONST"
      data: "2"
    }
  }
}
transit_list {
  start_node_name: "MoveToGear1"
  end_node_name: "GoToGetCalculate"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "MoveToGear1"
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
  start_node_name: "Path2"
  end_node_name: "MoveToGear1"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "terminated"
      module_name: "Path2"
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
  param_assignment {
    lhs_param {
      name: "gearIndex"
      module_name: "rootNode"
      type: "INT"
      category: "PLAN_VAR"
      robotUnit: "none"
    }
    rhs_param {
      type: "INT"
      category: "CONST"
      data: "3"
    }
  }
}
transit_list {
  start_node_name: "MoveToGear"
  end_node_name: "Path2"
  transit_period: 0
  trigger_condition {
    condition_type: "AND"
    trigger_condition {
      condition_type: "EQUAL"
      lhs_param {
        name: "reachedTarget"
        module_name: "MoveToGear"
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
      condition_type: "EQUAL"
      lhs_param {
        name: "gearIndex"
        module_name: "rootNode"
        type: "INT"
        category: "PLAN_VAR"
        robotUnit: "none"
      }
      rhs_param {
        type: "INT"
        category: "CONST"
        data: "2"
      }
    }
  }
}
transit_list {
  start_node_name: "MoveToGear"
  end_node_name: "Path3"
  transit_period: 0
  trigger_condition {
    condition_type: "AND"
    trigger_condition {
      condition_type: "EQUAL"
      lhs_param {
        name: "reachedTarget"
        module_name: "MoveToGear"
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
      condition_type: "EQUAL"
      lhs_param {
        name: "gearIndex"
        module_name: "rootNode"
        type: "INT"
        category: "PLAN_VAR"
        robotUnit: "none"
      }
      rhs_param {
        type: "INT"
        category: "CONST"
        data: "3"
      }
    }
  }
}
transit_list {
  start_node_name: "MoveToStd"
  end_node_name: "Retreat1"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "MoveToStd"
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
  start_node_name: "Retreat1"
  end_node_name: "End0"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "reachedTarget"
      module_name: "Retreat1"
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
  start_node_name: "Path3"
  end_node_name: "MoveToStd"
  transit_period: 0
  trigger_condition {
    condition_type: "EQUAL"
    lhs_param {
      name: "terminated"
      module_name: "Path3"
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
  node_name: "Retreat"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 0
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "Retreat"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "-0.05"
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
      module_name: "Retreat"
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
      module_name: "Retreat"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.03"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "Retreat"
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
      module_name: "Retreat"
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
      module_name: "Retreat"
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
      module_name: "Retreat"
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
      module_name: "Retreat"
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
      module_name: "Retreat"
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
  node_name: "Ready"
  pt_name: "MoveJ"
  pt_type: "MOVEJ"
  switch_tcp_param {
    switch_tcp: false
    tool_name: ""
    tcp_index: 0
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "Ready"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "VEC_7d"
      category: "CONST"
      data: "-117.552"
      data: "-34.881"
      data: "-1.213"
      data: "61.142"
      data: "32.231"
      data: "-34.185"
      data: "18.113"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxJntVel"
      module_name: "Ready"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxJntVel"
      type: "VEC_7d"
      category: "CONST"
      data: "60"
      data: "60"
      data: "70"
      data: "70"
      data: "140"
      data: "140"
      data: "140"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxJntAcc"
      module_name: "Ready"
      type: "VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "maxJntAcc"
      type: "VEC_7d"
      category: "CONST"
      data: "360"
      data: "360"
      data: "420"
      data: "420"
      data: "840"
      data: "840"
      data: "840"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "Ready"
      type: "ARRAY_VEC_7d"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_VEC_7d"
      category: "CONST"
    }
  }
  param_assignment {
    lhs_param {
      name: "relativeToStart"
      module_name: "Ready"
      type: "BOOL"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "relativeToStart"
      type: "BOOL"
      category: "CONST"
      data: "0"
    }
  }
  param_assignment {
    lhs_param {
      name: "targetTolLevel"
      module_name: "Ready"
      type: "INT"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "INT"
      category: "CONST"
      data: "10"
    }
  }
}
node_list {
  node_name: "CaliForceSensor0"
  pt_name: "CaliForceSensor"
  pt_type: "CALIFORCESENSOR"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 1
  }
}
node_list {
  node_name: "Insert"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 0
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "Insert"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "-0.162054421"
      data: "-0.479862236"
      data: "0.321394598"
      data: "153.606348"
      data: "-43.5075495"
      data: "62.0532702"
      data: "WORLD"
      data: "WORLD_ORIGIN"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "Insert"
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
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "Insert"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.2"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "Insert"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "200"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxAcc"
      module_name: "Insert"
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
      module_name: "Insert"
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
      module_name: "Insert"
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
      module_name: "Insert"
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
      module_name: "Insert"
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
  node_name: "GetTCP"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 2
  }
}
node_list {
  node_name: "Calculate"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 2
  }
}
node_list {
  node_name: "MoveToGear"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveToGear"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "gearPose"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveToGear"
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
      module_name: "MoveToGear"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveToGear"
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
      module_name: "MoveToGear"
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
      module_name: "MoveToGear"
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
      module_name: "MoveToGear"
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
      module_name: "MoveToGear"
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
      module_name: "MoveToGear"
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
  node_name: "MoveToStd"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveToStd"
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
      data: "0"
      data: "VAR"
      data: "stdPose"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveToStd"
      type: "ARRAY_COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "ARRAY_COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "0"
      data: "VAR"
      data: "gearPose"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxVel"
      module_name: "MoveToStd"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveToStd"
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
      module_name: "MoveToStd"
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
      module_name: "MoveToStd"
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
      module_name: "MoveToStd"
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
      module_name: "MoveToStd"
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
      module_name: "MoveToStd"
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
  node_name: "MoveToGear1"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 2
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "MoveToGear1"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      name: "gearPose"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
  }
  param_assignment {
    lhs_param {
      name: "waypoints"
      module_name: "MoveToGear1"
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
      module_name: "MoveToGear1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "MoveToGear1"
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
      module_name: "MoveToGear1"
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
      module_name: "MoveToGear1"
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
      module_name: "MoveToGear1"
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
      module_name: "MoveToGear1"
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
      module_name: "MoveToGear1"
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
  node_name: "GoToGetCalculate"
  pt_name: "GoTo"
  pt_type: "GOTO"
  switch_tcp_param {
    switch_tcp: false
    tool_name: ""
    tcp_index: 0
  }
  param_assignment {
    lhs_param {
      name: "nodeName"
      module_name: "GoToGetCalculate"
      type: "STRING"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "STRING"
      category: "CONST"
      data: "Calculate"
    }
  }
}
node_list {
  node_name: "Retreat1"
  pt_name: "MoveL"
  pt_type: "MOVEL"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "KostalTool"
    tcp_index: 0
  }
  param_assignment {
    lhs_param {
      name: "target"
      module_name: "Retreat1"
      type: "COORD"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "COORD"
      category: "CONST"
      data: "0"
      data: "0"
      data: "-0.1"
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
      module_name: "Retreat1"
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
      module_name: "Retreat1"
      type: "DOUBLE"
      category: "PT_INPUT"
    }
    rhs_param {
      type: "DOUBLE"
      category: "CONST"
      data: "0.5"
    }
  }
  param_assignment {
    lhs_param {
      name: "maxRotVel"
      module_name: "Retreat1"
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
      module_name: "Retreat1"
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
      module_name: "Retreat1"
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
      module_name: "Retreat1"
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
      module_name: "Retreat1"
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
      module_name: "Retreat1"
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
tool_names: "KostalTool"
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
var_list {
  name: "gearPoseOffset"
  module_name: "rootNode"
  type: "ARRAY_COORD"
  category: "PLAN_VAR"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "WORLD"
  data: "WORLD_ORIGIN"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "WORLD"
  data: "WORLD_ORIGIN"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "9.5"
  data: "WORLD"
  data: "WORLD_ORIGIN"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "19"
  data: "WORLD"
  data: "WORLD_ORIGIN"
  robotUnit: "m-deg"
}
var_list {
  name: "gearPose"
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
var_list {
  name: "gearIndex"
  module_name: "rootNode"
  type: "INT"
  category: "PLAN_VAR"
  data: "0"
  robotUnit: "none"
}
child_plans {
  plan_name: "LeftPath-MainPlan0"
  config {
    node_name: "Path1"
    pt_name: "Plan"
    pt_type: "PLAN"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  transit_list {
    start_node_name: "MoveUp1"
    end_node_name: "MoveDown1"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveUp1"
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
          name: "CartesianMomentY"
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
    start_node_name: "MoveDown1"
    end_node_name: "Stop"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveDown1"
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
          name: "CartesianMomentY"
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
    start_node_name: "MoveToStart"
    end_node_name: "Start"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveToStart"
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
          name: "CartesianMomentY"
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
    start_node_name: "Start"
    end_node_name: "MoveUp1"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Start"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  transit_list {
    start_node_name: "startNode"
    end_node_name: "MoveToStart"
    transit_period: 0
    trigger_condition {
      condition_type: "NO_CHECK"
    }
    param_assignment {
      lhs_param {
        name: "stdPose"
        module_name: "Path1"
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
    start_node_name: "Stop"
    end_node_name: "End0"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Stop"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  node_list {
    node_name: "MoveToStart"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveToStart"
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
        data: "20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
    node_name: "Start"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  node_list {
    node_name: "MoveUp1"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveUp1"
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
        data: "-20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveUp1"
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
        module_name: "MoveUp1"
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
        module_name: "MoveUp1"
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
        module_name: "MoveUp1"
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
        module_name: "MoveUp1"
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
        module_name: "MoveUp1"
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
        module_name: "MoveUp1"
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
        module_name: "MoveUp1"
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
    node_name: "MoveDown1"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveDown1"
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
        data: "20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveDown1"
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
        module_name: "MoveDown1"
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
        module_name: "MoveDown1"
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
        module_name: "MoveDown1"
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
        module_name: "MoveDown1"
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
        module_name: "MoveDown1"
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
        module_name: "MoveDown1"
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
        module_name: "MoveDown1"
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
    node_name: "Stop"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
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
  tool_names: "KostalTool"
  var_list {
    name: "stdPose"
    module_name: "Path1"
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
    enable_log: false
    time_interval: 5
    max_duration: 30
  }
  plan_desc: ""
}
child_plans {
  plan_name: "LeftPath-MainPlan01"
  config {
    node_name: "PathMain"
    pt_name: "Plan"
    pt_type: "PLAN"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  transit_list {
    start_node_name: "MoveUpMain"
    end_node_name: "MoveDownMain"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveUpMain"
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
        condition_type: "LESS_EQUAL"
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
    start_node_name: "MoveDownMain"
    end_node_name: "Stop"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveDownMain"
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
        condition_type: "GREATER_EQUAL"
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
    start_node_name: "MoveToStart"
    end_node_name: "Start"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveToStart"
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
        condition_type: "GREATER_EQUAL"
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
    start_node_name: "Start"
    end_node_name: "MoveUpMain"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Start"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  transit_list {
    start_node_name: "startNode"
    end_node_name: "MoveToStart"
    transit_period: 0
    trigger_condition {
      condition_type: "NO_CHECK"
    }
    param_assignment {
      lhs_param {
        name: "stdPose"
        module_name: "PathMain"
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
    start_node_name: "Stop"
    end_node_name: "End0"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Stop"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  node_list {
    node_name: "MoveToStart"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
    node_name: "Start"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  node_list {
    node_name: "MoveUpMain"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
        module_name: "MoveUpMain"
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
    node_name: "MoveDownMain"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
        module_name: "MoveDownMain"
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
    node_name: "Stop"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
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
  tool_names: "KostalTool"
  var_list {
    name: "stdPose"
    module_name: "PathMain"
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
    enable_log: false
    time_interval: 5
    max_duration: 30
  }
  plan_desc: ""
}
child_plans {
  plan_name: "LeftPath-MainPlan02"
  config {
    node_name: "Path2"
    pt_name: "Plan"
    pt_type: "PLAN"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  transit_list {
    start_node_name: "MoveUp2"
    end_node_name: "MoveDown2"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveUp2"
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
          name: "CartesianMomentY"
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
    start_node_name: "MoveDown2"
    end_node_name: "Stop"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveDown2"
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
          name: "CartesianMomentY"
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
    start_node_name: "MoveToStart"
    end_node_name: "Start"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveToStart"
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
          name: "CartesianMomentY"
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
    start_node_name: "Start"
    end_node_name: "MoveUp2"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Start"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  transit_list {
    start_node_name: "startNode"
    end_node_name: "MoveToStart"
    transit_period: 0
    trigger_condition {
      condition_type: "NO_CHECK"
    }
    param_assignment {
      lhs_param {
        name: "stdPose"
        module_name: "Path2"
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
    start_node_name: "Stop"
    end_node_name: "End0"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Stop"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  node_list {
    node_name: "MoveToStart"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveToStart"
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
        data: "20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
    node_name: "Start"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  node_list {
    node_name: "MoveUp2"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveUp2"
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
        data: "-20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveUp2"
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
        module_name: "MoveUp2"
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
        module_name: "MoveUp2"
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
        module_name: "MoveUp2"
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
        module_name: "MoveUp2"
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
        module_name: "MoveUp2"
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
        module_name: "MoveUp2"
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
        module_name: "MoveUp2"
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
    node_name: "MoveDown2"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveDown2"
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
        data: "20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveDown2"
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
        module_name: "MoveDown2"
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
        module_name: "MoveDown2"
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
        module_name: "MoveDown2"
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
        module_name: "MoveDown2"
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
        module_name: "MoveDown2"
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
        module_name: "MoveDown2"
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
        module_name: "MoveDown2"
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
    node_name: "Stop"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
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
  tool_names: "KostalTool"
  var_list {
    name: "stdPose"
    module_name: "Path2"
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
    enable_log: false
    time_interval: 5
    max_duration: 30
  }
  plan_desc: ""
}
child_plans {
  plan_name: "LeftPath-MainPlan021"
  config {
    node_name: "Path3"
    pt_name: "Plan"
    pt_type: "PLAN"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  transit_list {
    start_node_name: "MoveUp3"
    end_node_name: "MoveDown3"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveUp3"
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
          name: "CartesianMomentY"
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
    start_node_name: "MoveDown3"
    end_node_name: "Stop"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveDown3"
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
          name: "CartesianMomentY"
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
    start_node_name: "MoveToStart"
    end_node_name: "Start"
    transit_period: 0
    trigger_condition {
      condition_type: "OR"
      trigger_condition {
        condition_type: "EQUAL"
        lhs_param {
          name: "reachedTarget"
          module_name: "MoveToStart"
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
          name: "CartesianMomentY"
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
    start_node_name: "Start"
    end_node_name: "MoveUp3"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Start"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  transit_list {
    start_node_name: "startNode"
    end_node_name: "MoveToStart"
    transit_period: 0
    trigger_condition {
      condition_type: "NO_CHECK"
    }
    param_assignment {
      lhs_param {
        name: "stdPose"
        module_name: "Path3"
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
    start_node_name: "Stop"
    end_node_name: "End0"
    transit_period: 0
    trigger_condition {
      condition_type: "GREATER"
      lhs_param {
        name: "timePeriod"
        module_name: "Stop"
        type: "DOUBLE"
        category: "PT_STATE"
        robotUnit: "s"
      }
      rhs_param {
        type: "DOUBLE"
        category: "CONST"
        data: "0.1"
      }
    }
  }
  node_list {
    node_name: "MoveToStart"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveToStart"
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
        data: "20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
        module_name: "MoveToStart"
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
    node_name: "Start"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
  }
  node_list {
    node_name: "MoveUp3"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveUp3"
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
        data: "-20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveUp3"
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
        module_name: "MoveUp3"
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
        module_name: "MoveUp3"
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
        module_name: "MoveUp3"
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
        module_name: "MoveUp3"
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
        module_name: "MoveUp3"
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
        module_name: "MoveUp3"
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
        module_name: "MoveUp3"
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
    node_name: "MoveDown3"
    pt_name: "MoveL"
    pt_type: "MOVEL"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
    }
    param_assignment {
      lhs_param {
        name: "target"
        module_name: "MoveDown3"
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
        data: "20"
        data: "0"
        data: "VAR"
        data: "stdPose"
      }
    }
    param_assignment {
      lhs_param {
        name: "waypoints"
        module_name: "MoveDown3"
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
        module_name: "MoveDown3"
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
        module_name: "MoveDown3"
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
        module_name: "MoveDown3"
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
        module_name: "MoveDown3"
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
        module_name: "MoveDown3"
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
        module_name: "MoveDown3"
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
        module_name: "MoveDown3"
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
    node_name: "Stop"
    pt_name: "Hold"
    pt_type: "HOLD"
    switch_tcp_param {
      switch_tcp: false
      tool_name: ""
      tcp_index: 0
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
  tool_names: "KostalTool"
  var_list {
    name: "stdPose"
    module_name: "Path3"
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
    enable_log: false
    time_interval: 5
    max_duration: 30
  }
  plan_desc: ""
}
plan_log {
  enable_log: true
  time_interval: 1
  max_duration: 30
}
plan_desc: ""
                                                                                                                                                                                                                                          Project/LeftPath.proj                                                                               000644  000000  000000  00000007651 14250030100 013227  0                                                                                                    ustar 00                                                                000000  000000                                                                                                                                                                         project_name: "LeftPath"
version: "v2.8-develop"
updated_date: "2022-06-08_13-14-40"
plan_file_name: "LeftPath-MainPlan.plan"
scene_width: 3500
scene_height: 2106
plan_config {
  plan_name: "LeftPath-MainPlan"
  parent_node_name: ""
  node_config {
    node_name: "rootNode"
    x: 0
    y: 0
  }
  block_list {
    node_name: "startNode"
    x: 77
    y: 231
  }
  block_list {
    node_name: "Retreat"
    x: 357
    y: 231
  }
  block_list {
    node_name: "Ready"
    x: 637
    y: 231
  }
  block_list {
    node_name: "CaliForceSensor0"
    x: 917
    y: 231
  }
  block_list {
    node_name: "Insert"
    x: 1197
    y: 231
  }
  block_list {
    node_name: "GetTCP"
    x: 1477
    y: 231
  }
  block_list {
    node_name: "Calculate"
    x: 385
    y: 385
  }
  block_list {
    node_name: "MoveToGear"
    x: 616
    y: 385
  }
  block_list {
    node_name: "MoveToStd"
    x: 1232
    y: 847
  }
  block_list {
    node_name: "MoveToGear1"
    x: 1232
    y: 693
  }
  block_list {
    node_name: "GoToGetCalculate"
    x: 1463
    y: 693
  }
  block_list {
    node_name: "Retreat1"
    x: 1463
    y: 847
  }
  block_list {
    node_name: "End0"
    x: 1694
    y: 847
  }
  child_plan_list {
    plan_name: "LeftPath-MainPlan0"
    parent_node_name: "rootNode"
    node_config {
      node_name: "Path1"
      x: 924
      y: 539
    }
    block_list {
      node_name: "startNode"
      x: 77
      y: 231
    }
    block_list {
      node_name: "MoveToStart"
      x: 357
      y: 231
    }
    block_list {
      node_name: "Start"
      x: 637
      y: 231
    }
    block_list {
      node_name: "MoveUp1"
      x: 917
      y: 231
    }
    block_list {
      node_name: "MoveDown1"
      x: 1197
      y: 231
    }
    block_list {
      node_name: "Stop"
      x: 1477
      y: 231
    }
    block_list {
      node_name: "End0"
      x: 1771
      y: 231
    }
  }
  child_plan_list {
    plan_name: "LeftPath-MainPlan01"
    parent_node_name: "rootNode"
    node_config {
      node_name: "PathMain"
      x: 924
      y: 385
    }
    block_list {
      node_name: "startNode"
      x: 77
      y: 231
    }
    block_list {
      node_name: "MoveToStart"
      x: 357
      y: 231
    }
    block_list {
      node_name: "Start"
      x: 637
      y: 231
    }
    block_list {
      node_name: "MoveUpMain"
      x: 924
      y: 231
    }
    block_list {
      node_name: "MoveDownMain"
      x: 1155
      y: 231
    }
    block_list {
      node_name: "Stop"
      x: 1477
      y: 231
    }
    block_list {
      node_name: "End0"
      x: 1771
      y: 231
    }
  }
  child_plan_list {
    plan_name: "LeftPath-MainPlan02"
    parent_node_name: "rootNode"
    node_config {
      node_name: "Path2"
      x: 924
      y: 693
    }
    block_list {
      node_name: "startNode"
      x: 77
      y: 231
    }
    block_list {
      node_name: "MoveToStart"
      x: 357
      y: 231
    }
    block_list {
      node_name: "Start"
      x: 637
      y: 231
    }
    block_list {
      node_name: "MoveUp2"
      x: 917
      y: 231
    }
    block_list {
      node_name: "MoveDown2"
      x: 1197
      y: 231
    }
    block_list {
      node_name: "Stop"
      x: 1477
      y: 231
    }
    block_list {
      node_name: "End0"
      x: 1694
      y: 231
    }
  }
  child_plan_list {
    plan_name: "LeftPath-MainPlan021"
    parent_node_name: "rootNode"
    node_config {
      node_name: "Path3"
      x: 924
      y: 847
    }
    block_list {
      node_name: "startNode"
      x: 77
      y: 231
    }
    block_list {
      node_name: "MoveToStart"
      x: 357
      y: 231
    }
    block_list {
      node_name: "Start"
      x: 637
      y: 231
    }
    block_list {
      node_name: "MoveUp3"
      x: 917
      y: 231
    }
    block_list {
      node_name: "MoveDown3"
      x: 1197
      y: 231
    }
    block_list {
      node_name: "Stop"
      x: 1477
      y: 231
    }
    block_list {
      node_name: "End0"
      x: 1694
      y: 231
    }
  }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       