
const double updatesPerSecond = 10;
const double timeLimit = 1 / updatesPerSecond;
double timeElapsed = 0;
const double rotateSpeed = 1.0; // max rpm
List<IMyShipController> shipControllers = new List<IMyShipController>();
List<IMyMotorSuspension> suspensions = new List<IMyMotorSuspension>();

bool cruise = false;
bool autoDrill = false;
bool detected = false;
double groundDistance = 0.0;
const double defaultCruiseSpeed = 1.2;
float rotSpeed = 0.0f;

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
        case "cruise":
            cruise = !cruise;
            break;
		case "autodrill":
            autoDrill = !autoDrill;
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
	double armInput = referenceBlock.RollIndicator;
	
	// get rotors controlling the arm
	//IMyMotorStator rotorL	= GridTerminalSystem.GetBlockWithName("Advanced Rotor L") as IMyMotorStator;
	IMyMotorStator hingeRT = GridTerminalSystem.GetBlockWithName("Hinge Rear Top") as IMyMotorStator;
	
	if(autoDrill){
		IMyCameraBlock camera = GridTerminalSystem.GetBlockWithName("Camera Drill Raycast") as IMyCameraBlock;
		camera.EnableRaycast = true;
		if(camera.CanScan(10)){
			MyDetectedEntityInfo detectedObj = camera.Raycast(10,-35,0);
			detected = detectedObj.HitPosition.HasValue;
			if(detected){
				groundDistance = Vector3D.Distance(camera.GetPosition(), detectedObj.HitPosition.Value);
			} else {
				groundDistance = 999.0;
			}
			
		} else {
			//
		}
	}
	
	if(!autoDrill){
		hingeRT.TargetVelocityRPM = (float)(armInput * rotateSpeed);
	} else {
		double groundDistanceClamped = MathHelper.Clamp(groundDistance, 2.0, 4.0);
		rotSpeed = (float)(groundDistanceClamped - 3.0);
		if(groundDistanceClamped > 3.0){
			rotSpeed = rotSpeed/3.0f;
		}
		hingeRT.TargetVelocityRPM = (float)(rotSpeed * rotateSpeed);
	}
	
	///////////////////////////////////////////////////////
	
	GridTerminalSystem.GetBlocksOfType(suspensions);
	
	float speed = (float)referenceBlock.GetShipSpeed();
	float cruiseSpeed = (float)defaultCruiseSpeed;
	bool brakes = speed > (cruiseSpeed);
	speed = cruiseSpeed - (float)MathHelper.Clamp(speed, 0.0, cruiseSpeed);
	
	
	foreach (IMyMotorSuspension suspension in suspensions){		
		if(cruise){
			if(suspension.Orientation.Up.ToString() == "Left"){
				suspension.InvertPropulsion = true;
			}
			suspension.SetValueFloat("Propulsion override", -speed);
			referenceBlock.HandBrake = brakes;
		} else {
			suspension.SetValueFloat("Propulsion override", 0.0f);
			suspension.InvertPropulsion = false;

		}
	}
	
	IMyTextSurfaceProvider outputBlock;
	IMyTextSurface outputSurface;
	outputBlock	= GridTerminalSystem.GetBlockWithName("Cockpit") as IMyTextSurfaceProvider;
	outputSurface = outputBlock.GetSurface(2);
	//outputSurface = Me.GetSurface(0);
	outputSurface.ContentType = ContentType.TEXT_AND_IMAGE;
	outputSurface.FontSize = 1.2F;
	outputSurface.Alignment = VRage.Game.GUI.TextPanel.TextAlignment.CENTER;
	string debugText = "arm input = " + armInput
	 + "\n rotor velocity = " + hingeRT.TargetVelocityRPM
	 + "\n cruise control: " + cruise 
	 + "\n auto drill control: " + autoDrill 
	 + "\n surface detected: " + detected 
	 + "\n ground distance: " + groundDistance.ToString("0.00")
	 + "\n rot speed: " + rotSpeed.ToString("0.00")
	 + "\n EOF";
	outputSurface.WriteText(debugText);
	Echo(debugText);
	
}