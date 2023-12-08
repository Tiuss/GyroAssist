float power=0.0f;
double cruiseSpeed=0.0;
bool cruiseActive = false;
List<IMyShipController> shipControllers = new List<IMyShipController>();
List<IMyMotorSuspension> wheelsList = new List<IMyMotorSuspension>();
List<IMyMotorSuspension> wheelsLeftList = new List<IMyMotorSuspension>();
List<IMyMotorSuspension> wheelsRightList = new List<IMyMotorSuspension>();
IMyRemoteControl trainRemote;
List<float> throttleSets = new List<float> {0.0f,1.0f,5.0f,10.0f,15.0f,20.0f};
float reverse = 1.0f;
int throttleSet = 0;
bool referenceOnSameGridAsProgram = true; //if true, only searches for reference blocks on
                                          //the same grid as the program block (should help with docking small vessels)
bool keypress = false;

//---PID Constants
const double proportionalConstant = 1;
const double derivativeConstant = .1;
PID speedPID;

public Program()
{
     if ((Runtime.UpdateFrequency & UpdateFrequency.Update1) == 0)
			 Runtime.UpdateFrequency = UpdateFrequency.Update1;
			 
	List<IMyRemoteControl> remotes = new List<IMyRemoteControl>(); 
	GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remotes, remote => MyIni.HasSection(remote.CustomData, "train"));
	trainRemote = remotes.First();
	GridTerminalSystem.GetBlocksOfType<IMyMotorSuspension>(wheelsList, wheel => wheel.BlockDefinition.ToString() == "MyObjectBuilder_MotorSuspension/SmallShortSuspension3x3mirrored" || wheel.BlockDefinition.ToString() == "MyObjectBuilder_MotorSuspension/SmallShortSuspension3x3");
	var trainRight = trainRemote.WorldMatrix.Right;
	trainRight = Vector3D.TransformNormal(trainRight, trainRemote.WorldMatrix);
	foreach(var wheel in wheelsList){
		
		var wheelDir = Vector3D.TransformNormal(wheel.WorldMatrix.Up, trainRemote.WorldMatrix);
		if(trainRight.Dot(wheelDir) < 0){
			wheelsLeftList.Add(wheel);
		} else {
			wheelsRightList.Add(wheel);
		}
	
	}
	speedPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, 0.1);
}

public void Save()
{
	//
}

public void Main(string argument, UpdateType updateSource)
{
	 if ((Runtime.UpdateFrequency & UpdateFrequency.Update1) == 0)
		 Runtime.UpdateFrequency = UpdateFrequency.Update1;
	//string infoText = "TEST";
	
	 switch (argument.ToLower())
     {
        case "cruise":
		case "c":
             cruiseActive = !cruiseActive;
             break;
		case "reverse":
		case "r":
             reverse = -reverse;
             break;
		case "up":
		case "u":
             ThrottleUp();
             break;
		case "down":
		case "d":
             ThrottleDown();
             break;
	 }
	
	GridTerminalSystem.GetBlocksOfType(shipControllers, ShouldFetch);

    //---Check for any cases that would lead to code failure
    if (shipControllers.Count == 0)
    {
        // Echo($"ERROR: No ship controller was found");
        // return;
    }

    //---Assign our reference block
    IMyShipController shipController = GetControlledShipController(shipControllers);
	Vector3 kbInput = shipController.MoveIndicator;
	if(kbInput.Z == 0){
		power = 0f;
	} else if(kbInput.Z < 0) {
		power = -1f;
	} else {
		power = 1f;
	}
	
	if(kbInput.X == 0){	
		keypress = false;
	} else if(kbInput.X > 0) {
		if(!keypress) {
			ThrottleUp();
		}
		keypress = true;
	} else {
		if(!keypress){
			ThrottleDown();
		}
		keypress = true;
	}
		
	
	// List<IMyTerminalBlock> wheelsLeftList = new List<IMyTerminalBlock>();
	// List<IMyTerminalBlock> wheelsRightList = new List<IMyTerminalBlock>();
	// List<IMyTerminalBlock> wheelsMainList = new List<IMyTerminalBlock>();
	// IMyBlockGroup wheelsLeftGroup = GridTerminalSystem.GetBlockGroupWithName("Wheels Left");
	// IMyBlockGroup wheelsRightGroup = GridTerminalSystem.GetBlockGroupWithName("Wheels Right");
	// //IMyBlockGroup wheelsMainGroup = GridTerminalSystem.GetBlockGroupWithName("Wheels Main");
	// wheelsLeftGroup.GetBlocks(wheelsLeftList);
	// wheelsRightGroup.GetBlocks(wheelsRightList);
	// //wheelsMainGroup.GetBlocks(wheelsMainList);
	
	cruiseSpeed = reverse * throttleSets[throttleSet];
	var speed = shipController.GetShipSpeed();
	
	var sign = trainRemote.WorldMatrix.Forward.Dot(trainRemote.GetShipVelocities().LinearVelocity) > 0 ? -1 : 1;
	var diff = cruiseSpeed + speed*sign;
	var control = speedPID.Control(diff);
	var overrideAmount = MathHelper.Clamp(control/2,-1,1);
	
	
	
	
	if(Math.Abs(speed)<0.1 && throttleSet == 0 && kbInput.Z == 0){
		// full stop
		foreach(IMyMotorSuspension wheel in wheelsList){
			wheel.PropulsionOverride = 0.0f;
			wheel.Brake = true;
		}
	} else {
		// operation
		
		foreach (IMyMotorSuspension wheel in wheelsLeftList){
			wheel.Brake = false;
			if(cruiseActive)
				wheel.PropulsionOverride = (float)overrideAmount;
			else	
				wheel.PropulsionOverride = -power;
				
			bool wheelInvert = wheel.BlockDefinition.ToString() == "MyObjectBuilder_MotorSuspension/SmallShortSuspension3x3mirrored";
			if(wheelInvert)
				wheel.PropulsionOverride = -wheel.PropulsionOverride;
		}
		foreach (IMyMotorSuspension wheel in wheelsRightList){wheel.Brake = false;
			if(cruiseActive)
				wheel.PropulsionOverride = (float)overrideAmount;
			else	
				wheel.PropulsionOverride = -power;
				
			bool wheelInvert = wheel.BlockDefinition.ToString() == "MyObjectBuilder_MotorSuspension/SmallShortSuspension3x3";
			if(wheelInvert)
				wheel.PropulsionOverride = -wheel.PropulsionOverride;
		}
	}
	
	// if(kbInput.Y == 0){
		// foreach (IMyMotorSuspension wheel in wheelsMainList)
		// {
			// wheel.IsParkingEnabled = true;
		// }
		
	// }
	IMyTextSurface outputSurface = Me.GetSurface(0);
	outputSurface.ContentType = ContentType.TEXT_AND_IMAGE;
	outputSurface.FontSize = 1.2F;
	outputSurface.Alignment = VRage.Game.GUI.TextPanel.TextAlignment.CENTER;
	
	string debugText = ""
	+ "\n" + wheelsLeftList.Count().ToString()
	+ "\n" + wheelsRightList.Count().ToString()
	+ "\n" + wheelsList.Count().ToString()
	+ "\n cruise " + Math.Round(cruiseSpeed,2)
	+ "\n throttle " + throttleSet.ToString()
	//+ "\n diff" + Math.Round(diff,2)
	//+ "\n control" + Math.Round(control,2)
	+ "\n over " + Math.Round(overrideAmount,2)
	;
	if(Math.Abs(speed)<0.1 && throttleSet == 0)
		debugText += "\n park";
	outputSurface.WriteText(debugText);
	Echo(debugText);
	//Echo(power.ToString());
}

void ThrottleUp()
{
	if(throttleSet+1 <throttleSets.Count())
		++throttleSet;
}

void ThrottleDown()
{
	if(throttleSet > 0)
	--throttleSet;
}

bool ShouldFetch(IMyTerminalBlock block)
{
    if (block is IMyShipController)
    {
        if (referenceOnSameGridAsProgram)
        {
            return block.CubeGrid == Me.CubeGrid;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
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
	
	// if(Me.CustomData.Length > 0){
		// Echo(Me.CustomData);
		// return (IMyShipController)GridTerminalSystem.GetBlockWithName(Me.CustomData);
		
	// }
	
    return controllers[0];
}

//Whip's PID controller class v6 - 11/22/17
public class PID
{
    double _kP = 0;
    double _kI = 0;
    double _kD = 0;
    double _integralDecayRatio = 0;
    double _lowerBound = 0;
    double _upperBound = 0;
    double _timeStep = 0;
    double _inverseTimeStep = 0;
    double _errorSum = 0;
    double _lastError = 0;
    bool _firstRun = true;
    bool _integralDecay = false;
    public double Value { get; private set; }

    public PID(double kP, double kI, double kD, double lowerBound, double upperBound, double timeStep)
    {
        _kP = kP;
        _kI = kI;
        _kD = kD;
        _lowerBound = lowerBound;
        _upperBound = upperBound;
        _timeStep = timeStep;
        _inverseTimeStep = 1 / _timeStep;
        _integralDecay = false;
    }

    public PID(double kP, double kI, double kD, double integralDecayRatio, double timeStep)
    {
        _kP = kP;
        _kI = kI;
        _kD = kD;
        _timeStep = timeStep;
        _inverseTimeStep = 1 / _timeStep;
        _integralDecayRatio = integralDecayRatio;
        _integralDecay = true;
    }

    public double Control(double error)
    {
        //Compute derivative term
        var errorDerivative = (error - _lastError) * _inverseTimeStep;

        if (_firstRun)
        {
            errorDerivative = 0;
            _firstRun = false;
        }

        //Compute integral term
        if (!_integralDecay)
        {
            _errorSum += error * _timeStep;

            //Clamp integral term
            if (_errorSum > _upperBound)
                _errorSum = _upperBound;
            else if (_errorSum < _lowerBound)
                _errorSum = _lowerBound;
        }
        else
        {
            _errorSum = _errorSum * (1.0 - _integralDecayRatio) + error * _timeStep;
        }

        //Store this error as last error
        _lastError = error;

        //Construct output
        this.Value = _kP * error + _kI * _errorSum + _kD * errorDerivative;
        return this.Value;
    }
    
    public double Control(double error, double timeStep)
    {
        _timeStep = timeStep;
        _inverseTimeStep = 1 / _timeStep;
        return Control(error);
    }

    public void Reset()
    {
        _errorSum = 0;
        _lastError = 0;
        _firstRun = true;
    }
}