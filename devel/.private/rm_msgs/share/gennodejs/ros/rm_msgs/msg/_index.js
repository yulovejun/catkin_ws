
"use strict";

let Arm_IO_State = require('./Arm_IO_State.js');
let MoveJ_P = require('./MoveJ_P.js');
let Hand_Speed = require('./Hand_Speed.js');
let Force_Position_Move_Joint = require('./Force_Position_Move_Joint.js');
let ObjectInfo = require('./ObjectInfo.js');
let Hand_Seq = require('./Hand_Seq.js');
let MoveC = require('./MoveC.js');
let Hand_Posture = require('./Hand_Posture.js');
let MoveL = require('./MoveL.js');
let Arm_Analog_Output = require('./Arm_Analog_Output.js');
let Arm_Digital_Output = require('./Arm_Digital_Output.js');
let MoveJ = require('./MoveJ.js');
let Plan_State = require('./Plan_State.js');
let Lift_Height = require('./Lift_Height.js');
let Joint_Current = require('./Joint_Current.js');
let ChangeWorkFrame_Name = require('./ChangeWorkFrame_Name.js');
let Turtle_Driver = require('./Turtle_Driver.js');
let CartePos = require('./CartePos.js');
let Hand_Angle = require('./Hand_Angle.js');
let Joint_Step = require('./Joint_Step.js');
let ChangeTool_Name = require('./ChangeTool_Name.js');
let Stop = require('./Stop.js');
let ArmState = require('./ArmState.js');
let LiftState = require('./LiftState.js');
let Socket_Command = require('./Socket_Command.js');
let JointPos = require('./JointPos.js');
let Tool_Digital_Output = require('./Tool_Digital_Output.js');
let Gripper_Set = require('./Gripper_Set.js');
let Set_Force_Position = require('./Set_Force_Position.js');
let Arm_Current_State = require('./Arm_Current_State.js');
let Stop_Teach = require('./Stop_Teach.js');
let Joint_Enable = require('./Joint_Enable.js');
let Force_Position_Move_Pose = require('./Force_Position_Move_Pose.js');
let Hand_Force = require('./Hand_Force.js');
let Joint_Teach = require('./Joint_Teach.js');
let Tool_Analog_Output = require('./Tool_Analog_Output.js');
let Gripper_Pick = require('./Gripper_Pick.js');
let Lift_Speed = require('./Lift_Speed.js');
let Force_Position_State = require('./Force_Position_State.js');
let GetArmState_Command = require('./GetArmState_Command.js');
let Start_Multi_Drag_Teach = require('./Start_Multi_Drag_Teach.js');
let IO_Update = require('./IO_Update.js');
let Pos_Teach = require('./Pos_Teach.js');
let Ort_Teach = require('./Ort_Teach.js');
let Six_Force = require('./Six_Force.js');
let ChangeTool_State = require('./ChangeTool_State.js');
let Manual_Set_Force_Pose = require('./Manual_Set_Force_Pose.js');
let Tool_IO_State = require('./Tool_IO_State.js');
let ChangeWorkFrame_State = require('./ChangeWorkFrame_State.js');

module.exports = {
  Arm_IO_State: Arm_IO_State,
  MoveJ_P: MoveJ_P,
  Hand_Speed: Hand_Speed,
  Force_Position_Move_Joint: Force_Position_Move_Joint,
  ObjectInfo: ObjectInfo,
  Hand_Seq: Hand_Seq,
  MoveC: MoveC,
  Hand_Posture: Hand_Posture,
  MoveL: MoveL,
  Arm_Analog_Output: Arm_Analog_Output,
  Arm_Digital_Output: Arm_Digital_Output,
  MoveJ: MoveJ,
  Plan_State: Plan_State,
  Lift_Height: Lift_Height,
  Joint_Current: Joint_Current,
  ChangeWorkFrame_Name: ChangeWorkFrame_Name,
  Turtle_Driver: Turtle_Driver,
  CartePos: CartePos,
  Hand_Angle: Hand_Angle,
  Joint_Step: Joint_Step,
  ChangeTool_Name: ChangeTool_Name,
  Stop: Stop,
  ArmState: ArmState,
  LiftState: LiftState,
  Socket_Command: Socket_Command,
  JointPos: JointPos,
  Tool_Digital_Output: Tool_Digital_Output,
  Gripper_Set: Gripper_Set,
  Set_Force_Position: Set_Force_Position,
  Arm_Current_State: Arm_Current_State,
  Stop_Teach: Stop_Teach,
  Joint_Enable: Joint_Enable,
  Force_Position_Move_Pose: Force_Position_Move_Pose,
  Hand_Force: Hand_Force,
  Joint_Teach: Joint_Teach,
  Tool_Analog_Output: Tool_Analog_Output,
  Gripper_Pick: Gripper_Pick,
  Lift_Speed: Lift_Speed,
  Force_Position_State: Force_Position_State,
  GetArmState_Command: GetArmState_Command,
  Start_Multi_Drag_Teach: Start_Multi_Drag_Teach,
  IO_Update: IO_Update,
  Pos_Teach: Pos_Teach,
  Ort_Teach: Ort_Teach,
  Six_Force: Six_Force,
  ChangeTool_State: ChangeTool_State,
  Manual_Set_Force_Pose: Manual_Set_Force_Pose,
  Tool_IO_State: Tool_IO_State,
  ChangeWorkFrame_State: ChangeWorkFrame_State,
};
