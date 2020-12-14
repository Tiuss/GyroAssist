
const double updatesPerSecond = 10;
const double timeLimit = 1 / updatesPerSecond;
double timeElapsed = 0;
const double rotateSpeed = 2; // max rpm
const float traverseSpeed = 1f; // max speed 
List<IMyShipController> shipControllers = new List<IMyShipController>();
List<IMyTerminalBlock> nPistons = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> sPistons = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> ePistons = new List<IMyTerminalBlock>();
List<IMyTerminalBlock> wPistons = new List<IMyTerminalBlock>();

bool toggle = true;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Once;
    Echo("If you can read this\nclick the 'Run' button!");
}

void Main(string arg, UpdateType updateSource)
{
    if ((Runtime.UpdateFrequency & UpdateFrequency.Update1) == 0)
        Runtime.UpdateFrequency = UpdateFrequency.Update1;

    timeElapsed += Runtime.TimeSinceLastRun.TotalSeconds;

	switch (arg.ToLower())
    {
        case "toggle":
            toggle = !toggle;
            break;
		
		default:
		break;
	}

    if (timeElapsed >= timeLimit)
    {
        ControlArm();
        timeElapsed = 0;
    }
}

IMyShipController GetControlledShipController(List<IMyShipController> controllers)
{
    if (controllers.Count == 0)
        return null;
    
    foreach (IMyShipController thisController in controllers)
    {
        if (thisController.IsUnderControl && thisController.CanControlShip)
            return thisController;
    }
	
    return controllers[0];	
}

void ControlArm()
{
	//---Find our refrence and comparision blocks    
    GridTerminalSystem.GetBlocksOfType(shipControllers);

    //---Check for any cases that would lead to code failure
    if (shipControllers.Count == 0)
    {
        Echo($"ERROR: No ship controller was found");
        return;
    }

    //---Assign our reference block
    IMyShipController referenceBlock = GetControlledShipController(shipControllers);
	var moveInput = referenceBlock.MoveIndicator;
	double rotSpeed = referenceBlock.RollIndicator;
	// get rotors controlling the arm
	IMyMotorStator rotorCrane	= GridTerminalSystem.GetBlockWithName("Advanced Rotor Crane") as IMyMotorStator;
	IMyMotorStator rotorHead	= GridTerminalSystem.GetBlockWithName("Advanced Rotor Head") as IMyMotorStator;
	
	rotorCrane.TargetVelocityRPM = (float)(moveInput.X * rotateSpeed);
	rotorHead.TargetVelocityRPM = (float)(-rotSpeed * rotateSpeed);
	
	//IMyBlockGroup nGroup = GridTerminalSystem.GetBlockGroupWithName("N");
	IMyBlockGroup sGroup = GridTerminalSystem.GetBlockGroupWithName("S");
	IMyBlockGroup eGroup = GridTerminalSystem.GetBlockGroupWithName("E");
	IMyBlockGroup wGroup = GridTerminalSystem.GetBlockGroupWithName("W");
	//nGroup.GetBlocks(nPistons);
	sGroup.GetBlocks(sPistons);
	eGroup.GetBlocks(ePistons);
	wGroup.GetBlocks(wPistons);
	
	if(toggle){
		//foreach (var block in nPistons)
		//{
		//	((IMyExtendedPistonBase)block).Velocity = mouseInput.Z * traverseSpeed;
		//}
		foreach (var block in sPistons)
		{
			((IMyExtendedPistonBase)block).Velocity = moveInput.Y * traverseSpeed;
		}
		foreach (var block in ePistons)
		{
			((IMyExtendedPistonBase)block).Velocity = -moveInput.Z * traverseSpeed;
		}
		foreach (var block in wPistons)
		{
			((IMyExtendedPistonBase)block).Velocity = moveInput.Z * traverseSpeed;
		}
		//pistonCrane.Velocity = -mouseInput.Y * traverseSpeed;
	
	}
	
	///////////////////////////////////////////////////////

	
	//IMyTextSurfaceProvider outputBlock;
	IMyTextSurface outputSurface;
	//outputBlock	= GridTerminalSystem.GetBlockWithName("LCD") as IMyTextSurfaceProvider;
	//outputSurface = outputBlock.GetSurface(0);
	outputSurface = Me.GetSurface(0);
	outputSurface.ContentType = ContentType.TEXT_AND_IMAGE;
	outputSurface.FontSize = 1.2F;
	outputSurface.Alignment = VRage.Game.GUI.TextPanel.TextAlignment.CENTER;
	string debugText = "arm input = " + moveInput.X
	 + "\n enabled?: " + toggle 
	 + "\n EOF";
	outputSurface.WriteText(debugText);
	Echo(debugText);
	
}