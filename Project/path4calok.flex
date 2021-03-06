Project/path4calok-MainPlan.plan                                                                    000644  000000  000000  00000011151 14216022745 015237  0                                                                                                    ustar 00                                                                000000  000000                                                                                                                                                                         plan_name: "path4calok-MainPlan"
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
  start_node_name: "Hold0"
  end_node_name: "Hold2"
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
      data: "0.1"
    }
  }
  param_operation {
    operator_type: "MULTIPLICATION"
    in_param_a {
      name: "v2"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    in_param_b {
      name: "v3"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    out_param {
      name: "v2"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
  }
}
transit_list {
  start_node_name: "Hold2"
  end_node_name: "Hold1"
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
      data: "0.1"
    }
  }
  param_operation {
    operator_type: "INVERSE"
    in_param_a {
      name: "v1"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    out_param {
      name: "v1"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
  }
  param_operation {
    operator_type: "MULTIPLICATION"
    in_param_a {
      name: "v1"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    in_param_b {
      name: "v2"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
    out_param {
      name: "v1"
      module_name: "rootNode"
      type: "COORD"
      category: "PLAN_VAR"
      robotUnit: "m-deg"
    }
  }
}
transit_list {
  start_node_name: "startNode"
  end_node_name: "Hold3"
  transit_period: 0
  trigger_condition {
    condition_type: "NO_CHECK"
  }
}
transit_list {
  start_node_name: "Hold3"
  end_node_name: "Hold0"
  transit_period: 0
  trigger_condition {
    condition_type: "GREATER"
    lhs_param {
      name: "timePeriod"
      module_name: "Hold3"
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
      name: "v1"
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
node_list {
  node_name: "Hold0"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest"
    tcp_index: 3
  }
}
node_list {
  node_name: "Hold1"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: false
    tool_name: ""
    tcp_index: 0
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
  node_name: "Hold3"
  pt_name: "Hold"
  pt_type: "HOLD"
  switch_tcp_param {
    switch_tcp: true
    tool_name: "Tooltest"
    tcp_index: 1
  }
}
tool_names: "Tooltest"
tool_names: "Tooltest2"
var_list {
  name: "v1"
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
  name: "v2"
  module_name: "rootNode"
  type: "COORD"
  category: "PLAN_VAR"
  data: "-0.132099"
  data: "-0.469619"
  data: "0.103262"
  data: "44.705"
  data: "-0.364"
  data: "0.099"
  data: "WORLD"
  data: "WORLD_ORIGIN"
  robotUnit: "m-deg"
}
var_list {
  name: "v3"
  module_name: "rootNode"
  type: "COORD"
  category: "PLAN_VAR"
  data: "0.224"
  data: "0.1248"
  data: "0.1035"
  data: "0"
  data: "0"
  data: "3.48"
  data: "WORLD"
  data: "WORLD_ORIGIN"
  robotUnit: "m-deg"
}
var_list {
  name: "v4"
  module_name: "rootNode"
  type: "POSE"
  category: "PLAN_VAR"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "0"
  data: "21.78"
  robotUnit: "m-deg"
}
plan_log {
  enable_log: false
  time_interval: 5
  max_duration: 30
}
plan_desc: ""
                                                                                                                                                                                                                                                                                                                                                                                                                       Project/path4calok.proj                                                                             000644  000000  000000  00000001165 14216022745 013566  0                                                                                                    ustar 00                                                                000000  000000                                                                                                                                                                         project_name: "path4calok"
version: "v2.8-develop"
updated_date: "2022-03-21_15-14-45"
plan_file_name: "path4calok-MainPlan.plan"
scene_width: 3466
scene_height: 2260
plan_config {
  plan_name: "path4calok-MainPlan"
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
    node_name: "Hold0"
    x: 539
    y: 385
  }
  block_list {
    node_name: "Hold1"
    x: 1155
    y: 385
  }
  block_list {
    node_name: "Hold2"
    x: 847
    y: 385
  }
  block_list {
    node_name: "Hold3"
    x: 308
    y: 385
  }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           