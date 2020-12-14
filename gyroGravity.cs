/*
/// Whip's Gravity Alignment Systems v25 - revision: 5/22/18 ///    

Written by Whiplash141    
*/

/*
Heavy modified by Tiuss 
*/

/*    
==============================    
    You can edit these vars   
==============================  
*/

enum DampenersSetup : byte {On = 0, Cruise, Off};
enum AssistLinear : byte {On = 0, Forward, Reverse, Off};

const string gyroExcludeName = "Exclude";
const string statusScreenName = "Alignment"; //(Optional) Name of status screen

const string shipName = "\n         [SHIP NAME GOES HERE]"; //(Optional) Name of your ship

bool shouldAlign = true; //If the script should attempt to stabalize by default
AssistLinear assistLinear = AssistLinear.On; // If the grid should pitch down/up when W/S pressed
bool assistLateral = true; // If the grid should bank left/right when A/D pressed
DampenersSetup dampenersSetup = DampenersSetup.On;
bool referenceOnSameGridAsProgram = true; //if true, only searches for reference blocks on
                                          //the same grid as the program block (should help with docking small vessels)

//---PID Constants
const double proportionalConstant = 2;
const double derivativeConstant = .5;

const double agilityConstant = 1; // increase when more gyros than usual, decrease when less; // DEFAULT 1.0

/*  
====================================================  
    Don't touch anything below this <3 - Whiplash  
	Too late; had my way with this - Tiuss
====================================================  
*/
const double speedTreshold = 33; // max bank at this speed
const double maxAngle = 45; // max bank angle in degrees

const double updatesPerSecond = 10;
const double timeFlashMax = .5; //in seconds  
const double timeLimit = 1 / updatesPerSecond;
double angleRoll = 0;
double anglePitch = 0;
double timeElapsed = 0;
string gravityMagnitudeString;
string overrideStatus;

List<IMyGyro> gyros = new List<IMyGyro>();
List<IMyThrust> thrusters = new List<IMyThrust>();
List<IMyThrust> liftThrusters = new List<IMyThrust>();
List<IMyShipController> shipControllers = new List<IMyShipController>();

PID pitchPID;
PID rollPID;
PID vspeedPID;

MyIni _ini;

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Once;
    Echo("If you can read this\nclick the 'Run' button!");
    pitchPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, timeLimit);
    rollPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, timeLimit);
	vspeedPID = new PID(proportionalConstant, 0, derivativeConstant, -10, 10, timeLimit);
	_ini = new MyIni();
}

void Main(string arg, UpdateType updateSource)
{
    //------------------------------------------
    //This is a bandaid
    if ((Runtime.UpdateFrequency & UpdateFrequency.Update1) == 0)
        Runtime.UpdateFrequency = UpdateFrequency.Update1;
    //------------------------------------------
    
    timeElapsed += Runtime.TimeSinceLastRun.TotalSeconds;

    switch (arg.ToLower())
    {
        case "toggle":
            if (!shouldAlign)
            {
                shouldAlign = true;
            }
            else
            {
                shouldAlign = false;
            }
            break;

        case "on":
            shouldAlign = true;
            break;

        case "off":
            shouldAlign = false;
            break;
			
		case "dampeners":
			if(dampenersSetup == DampenersSetup.On) {dampenersSetup = DampenersSetup.Cruise;}
			else if (dampenersSetup == DampenersSetup.Cruise) {dampenersSetup = DampenersSetup.Off;}
			else {dampenersSetup = DampenersSetup.On;}
			break;
			
		case "linear":
			if(assistLinear == AssistLinear.On) {assistLinear = AssistLinear.Reverse;}
			else if (assistLinear == AssistLinear.Reverse) {assistLinear = AssistLinear.Forward;}
			else if (assistLinear == AssistLinear.Forward) {assistLinear = AssistLinear.Off;}
			else {assistLinear = AssistLinear.On;}
			break;
			
		case "lateral":
			assistLateral = !assistLateral;
			break;
        default:
			string[] storedData = Storage.Split(';');
			if(storedData.Length == 4){
				string dampenersString, linearString;
				shouldAlign = storedData[0] == "1";
				
				dampenersString = storedData[1];
				linearString = storedData[2];
				
				if(dampenersString == "0") {dampenersSetup = DampenersSetup.Off;}
				else if (dampenersString == "1") {dampenersSetup = DampenersSetup.On;}
				else {dampenersSetup = DampenersSetup.Cruise;}
				
				if(linearString == "0") {assistLinear = AssistLinear.Off;}
				else if (linearString == "1") {assistLinear = AssistLinear.On;}
				else if (linearString == "2") {assistLinear = AssistLinear.Reverse;}
				else {assistLinear = AssistLinear.Forward;}
				
				assistLateral = storedData[3] == "1";
			}
			Storage = "";
            break;
    }

    if (timeElapsed >= timeLimit)
    {
        AlignWithGravity();
        timeElapsed = 0;
    }
}

void Save(){
	string dampenersString, linearString;
	if(dampenersSetup == DampenersSetup.On) {dampenersString = "1";}
	else if (dampenersSetup == DampenersSetup.Cruise) {dampenersString = "2";}
	else {dampenersString = "0";}
	
	if(assistLinear == AssistLinear.On) {linearString = "1";}
	else if (assistLinear == AssistLinear.Reverse) {linearString = "2";}
	else if (assistLinear == AssistLinear.Forward) {linearString = "3";}
	else {linearString = "0";}
	Storage = string.Join(";",
		shouldAlign ? "1" : "0",
		dampenersString,
		linearString,
		assistLateral ? "1" : "0");
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

void AlignWithGravity()
{
    //---Find our refrence and comparision blocks    
    GridTerminalSystem.GetBlocksOfType(shipControllers, ShouldFetch);

    //---Check for any cases that would lead to code failure
    if (shipControllers.Count == 0)
    {
        Echo($"ERROR: No ship controller was found");
        return;
    }

    //---Assign our reference block
    IMyShipController referenceBlock = GetControlledShipController(shipControllers);

    //---Populate gyro and thruster list
    gyros.Clear();
	thrusters.Clear();
	liftThrusters.Clear();
	
    GridTerminalSystem.GetBlocksOfType(gyros, block => block.CubeGrid == referenceBlock.CubeGrid && !block.CustomName.Contains(gyroExcludeName));
	GridTerminalSystem.GetBlocksOfType(thrusters);
	thrusters = thrusters.Where(thruster => thruster.IsWorking == true).ToList();
	liftThrusters = thrusters.Where(thruster => thruster.Orientation.Forward.ToString() == "Down").ToList();

	if (gyros.Count == 0)
    {
        Echo("ERROR: No gyros found on ship");
        return;
    }
	
	if (liftThrusters.Count == 0)
    {
        Echo("ERROR: No lift thrusters found on ship");
		
		foreach (IMyGyro thisGyro in gyros)
        {
            thisGyro.SetValue("Override", false);
        }		
        return;
    }

    //---Get gravity vector    
    var referenceOrigin = referenceBlock.GetPosition();
    var gravityVec = referenceBlock.GetNaturalGravity();
    var gravityVecLength = gravityVec.Length();
    gravityMagnitudeString = Math.Round(gravityVecLength, 2).ToString() + " m/s²";
    if (gravityVec.LengthSquared() == 0)
    {
        gravityMagnitudeString = "No Gravity";

        foreach (IMyGyro thisGyro in gyros)
        {
            thisGyro.SetValue("Override", false);
        }
        overrideStatus = "";

        shouldAlign = false;

        angleRoll = 0; angleRoll = 0;
		Echo("ERROR: No gravity");
        return;
    }
	
	double maxAngleCalculated = maxAngle * Math.PI / 180;
	double maxLiftG = 2.0;
	double maxAccG = 1.0;
	double maxLiftAcc = 20.0;
	double speedTresholdCalculated = speedTreshold;
	double totalThrust = 0;
	double shipMass = 0;
    if (liftThrusters.Count != 0) 
    {
        // calculate max allowed bank angle based on available lift thrust vs ship weight		
		foreach (IMyThrust thruster in liftThrusters){
			totalThrust += thruster.MaxEffectiveThrust; 
		}
		shipMass = referenceBlock.CalculateShipMass().PhysicalMass;
		maxLiftAcc = totalThrust/shipMass;
		maxLiftG = maxLiftAcc/gravityVec.Length(); // max total acceleration created by engines (phrased in Gees but excluding gravity's effect, hence +1)
		maxAccG = maxLiftG - 1; // max upwards acceleration within gravity in Gees, taking gravity into account
		maxAngleCalculated = Math.Atan(maxLiftG);
		speedTresholdCalculated = 10 / agilityConstant * maxLiftG; 
    } else {
		foreach (IMyThrust thruster in thrusters){
			thruster.ThrustOverride = 0.0f;
		}
	}
	

    //---Dir'n vectors of the reference block     
    var referenceForward = referenceBlock.WorldMatrix.Forward;
    var referenceLeft = referenceBlock.WorldMatrix.Left;
    var referenceUp = referenceBlock.WorldMatrix.Up;

    //---Get Roll and Pitch Angles 
    anglePitch = Math.Acos(MathHelper.Clamp(gravityVec.Dot(referenceForward) / gravityVecLength, -1, 1)) - Math.PI / 2;

    Vector3D planetRelativeLeftVec = referenceForward.Cross(gravityVec);
	Vector3D planetRelativeFwdVec = -referenceLeft.Cross(gravityVec);
    angleRoll = VectorAngleBetween(referenceLeft, planetRelativeLeftVec);
    angleRoll *= VectorCompareDirection(VectorProjection(referenceLeft, gravityVec), gravityVec); //ccw is positive 

    anglePitch *= -1; angleRoll *= -1;

	// some initial values
	Vector3 kbInput = referenceBlock.MoveIndicator;
	double lateralAngleAmount = 0;
	double linearAngleAmount = 0;
	double lateralAngleCorrection = 0;
	double linearAngleCorrection = 0;
	double lateralSpeed = 0;
	double linearSpeed = 0;
	double verticalSpeed =0;
	Vector3D velocity = referenceBlock.GetShipVelocities().LinearVelocity;
	Vector3D verticalVec = Vector3D.ProjectOnVector(ref velocity,ref gravityVec);
	verticalSpeed = -1.0 * verticalVec.Length();
	verticalSpeed *= VectorCompareDirection(velocity, gravityVec);
	double vSpeed = 0.0;
	double vSpeedClamp = 0.0;
	bool useVerticalDampeners = false;
	
	// some intermediate lateral values
	
	// for roll correction, I need to calculate how much% of total velocity is lateral velocity. 
	// To do that, I project the ship velocity onto already available planet relative left vector, both which need to be normalized
	Vector3D planetRelativeLeftVecNormalized = Vector3D.Divide(planetRelativeLeftVec,planetRelativeLeftVec.Length());
	Vector3D lateral = Vector3D.ProjectOnVector(ref velocity,ref planetRelativeLeftVecNormalized);	
	// I now calculate the roll amount needed, depending on lateral speed and max bank allowed, both which are editable constants
	lateralSpeed = lateral.Length();	
	// right is negative, left positive
	lateralSpeed *= VectorCompareDirection(planetRelativeLeftVecNormalized, lateral);
	
	// some intermediate vertical lateral values
	double targetVspeed = 0.0;
	double verticalSpeedDifference = targetVspeed - verticalSpeed;
	vSpeed = vspeedPID.Control(verticalSpeedDifference);
	double maxAccSpeed = 5; // arbitrary value due to testing; higher is less agile, lower is unstable.
	vSpeedClamp = (MathHelper.Clamp(vSpeed,-maxAccSpeed,maxAccSpeed*(maxAccG))/maxAccSpeed + 1.0) / maxLiftG;
	
	// some intermediate linear lateral values	
	// same approach as for lateral
	Vector3D planetRelativeFwdVecNormalized = Vector3D.Divide(planetRelativeFwdVec,planetRelativeFwdVec.Length());
	Vector3D linear = Vector3D.ProjectOnVector(ref velocity,ref planetRelativeFwdVecNormalized);
	linearSpeed = linear.Length();
	// right is negative, left positive
	linearSpeed *= VectorCompareDirection(planetRelativeFwdVecNormalized, linear);
	
	// handle user left/right input depending on assist and dampeners config
	if(kbInput.X == 0){
		if(dampenersSetup == DampenersSetup.On || dampenersSetup == DampenersSetup.Cruise) {
			lateralAngleAmount = (MathHelper.Clamp(lateralSpeed,-speedTresholdCalculated,speedTresholdCalculated)) / speedTresholdCalculated; //  calculation of percentage of full bank within (-1,1)
		} else {
			lateralAngleAmount = 0.0;
		}
	} else {
		if(assistLateral){
			useVerticalDampeners = true;
			lateralAngleAmount = kbInput.X/2;
		} else {
			useVerticalDampeners = false;
			lateralAngleAmount = 0.0;
		}
	}
	
	// same for linear velocity and input
	if(kbInput.Z == 0){
		if(dampenersSetup == DampenersSetup.On) {
			linearAngleAmount = (MathHelper.Clamp(linearSpeed,-speedTresholdCalculated,speedTresholdCalculated)) / speedTresholdCalculated;
		} else {
			linearAngleAmount = 0.0;
		}
	} else if(kbInput.Z < 0) {
		if(assistLinear == AssistLinear.On || assistLinear == AssistLinear.Forward){
			useVerticalDampeners = true;
			linearAngleAmount = kbInput.Z/2;
		} else {
			useVerticalDampeners = false;
			linearAngleAmount = 0.0;
		}
	} else {
		if(assistLinear == AssistLinear.On || assistLinear == AssistLinear.Reverse){
			useVerticalDampeners = true;
			linearAngleAmount = kbInput.Z/2;
		} else {
			useVerticalDampeners = false;
			linearAngleAmount = 0.0;
		}
	}
	
	// attempt to maintain altitude if no user input & above some minimal horizontal speed value (my algorithm is too jittery at small speeds)
	// if(kbInput.Y == 0 && liftThrusters.Count != 0 && Math.Abs(lateralSpeed) + Math.Abs(linearSpeed) > speedTresholdCalculated*agilityConstant/5){
		// useVerticalDampeners = true;		
	//} else if (kbInput.Y != 0) < speedTresholdCalculated*agilityConstant/5 ) {
	if (kbInput.Y != 0 || Math.Abs(lateralSpeed) + Math.Abs(linearSpeed) < 2 ) {
		useVerticalDampeners = false;
	}
	
	if(useVerticalDampeners) {
		foreach (IMyThrust thruster in liftThrusters){	
			thruster.ThrustOverridePercentage = (float)vSpeedClamp;
		}
	} else {
		foreach (IMyThrust thruster in liftThrusters){
			thruster.ThrustOverride = 0.0f;
		}
	}

	// calculate the actual pitch/roll correction based on max angle possible
	lateralAngleCorrection = lateralAngleAmount * maxAngleCalculated;
	linearAngleCorrection = linearAngleAmount * maxAngleCalculated;
	lateralAngleCorrection = -lateralAngleCorrection + angleRoll;
	linearAngleCorrection = linearAngleCorrection + anglePitch;
	
    //---Angle controller    
    double rollSpeed = rollPID.Control(lateralAngleCorrection); //Math.Round(angleRoll * proportionalConstant + (angleRoll - lastAngleRoll) / timeElapsed * derivativeConstant, 2);
    double pitchSpeed = pitchPID.Control(linearAngleCorrection); //Math.Round(anglePitch * proportionalConstant + (anglePitch - lastAnglePitch) / timeElapsed * derivativeConstant, 2);

	//	---------------------------------------------
	// output on Programmable Block's screen
	string lateralStatusString = "";
	string linearStatusString = "";
	string dampenersStatusString = "";
	
	if(assistLateral){lateralStatusString = "On";} else {lateralStatusString = "Off";}
	if(assistLinear == AssistLinear.On){linearStatusString = "On";} 
	else if(assistLinear == AssistLinear.Forward) {linearStatusString = "Forward";}
	else if(assistLinear == AssistLinear.Reverse) {linearStatusString = "Reverse";}
	else {linearStatusString = "Off";}
	
	if(dampenersSetup == DampenersSetup.On) {dampenersStatusString = "Full";}
	else if(dampenersSetup == DampenersSetup.Cruise) {dampenersStatusString = "Cruise";}
	else {dampenersStatusString = "Off";}
	
	if(!shouldAlign) {
		lateralStatusString = "Off";
		linearStatusString = "Off";
		dampenersStatusString = "Off";
	}
	
	//output result to custom surface, this programmable block's screen as a default
	MyIniParseResult result;
    if (!_ini.TryParse(Me.CustomData, out result)) {
        throw new Exception(result.ToString());
	}
	string customOutputBlockName = "";
	int customSurfaceNumber = 0;
	customSurfaceNumber = _ini.Get("setup", "customSurfaceNumber").ToInt32(0);
	customOutputBlockName = _ini.Get("setup", "customBlockName").ToString("");
	
	IMyTextSurfaceProvider outputBlock;
	IMyTextSurface outputSurface;
	outputBlock	= GridTerminalSystem.GetBlockWithName(customOutputBlockName) as IMyTextSurfaceProvider;
	
	if(outputBlock == null){
		outputBlock = Me;
	}
	if(customSurfaceNumber >= outputBlock.SurfaceCount){
		customSurfaceNumber = 0; // make sure block has this surface
	}
	
	outputSurface = outputBlock.GetSurface(customSurfaceNumber);
	
	//IMyTextSurface outputSurface = Me.GetSurface(0);
	
	outputSurface.ContentType = ContentType.TEXT_AND_IMAGE;
	outputSurface.FontSize = 1.2F;
	outputSurface.Alignment = VRage.Game.GUI.TextPanel.TextAlignment.CENTER;
	string debugText = ""
	//+ "lat correction:" + Math.Round((lateralAngleCorrection / Math.PI * 180), 2).ToString() + "deg"
	//+ "\n lin correction:" + Math.Round((linearAngleCorrection / Math.PI * 180), 2).ToString() + "deg"
	+ "Lateral assist: " + lateralStatusString
	+ "\n Linear assist: " + linearStatusString
	+ "\n Dampener setup: " + dampenersStatusString
	+ "\n lat/lin speed: " + Math.Round(lateralSpeed,2) + " / " + Math.Round(linearSpeed,2)
	+ "\n max lift: " + Math.Round(maxLiftG,2) + "Gs (" + Math.Round(maxLiftAcc,2) + " m/s²)" 
	+ "\n max bank:" + Math.Round((maxAngleCalculated / Math.PI * 180), 1).ToString() + "deg at " + Math.Round(speedTresholdCalculated,1).ToString() + "m/s";
	outputSurface.WriteText(debugText);
	Echo(debugText);

	//	---------------------------------------------

    var mouseInput = referenceBlock.RotationIndicator;

    //---Set appropriate gyro override  
    if (shouldAlign)
    {
        //do gyros
		// allow some user input on pitch (looking/aiming etc)
		// I have to reorient the pitch to include some yaw component when the ship is banked to avoid unwanted rotation (both manually & via correction)
		// I don't speak quaternions so doing it the trigonometry way
		//double pitch = MathHelper.Clamp((pitchSpeed - mouseInput.X), -1, 1);
		double pitch = pitchSpeed;
		double yaw = 0.0;
		double roll = -rollSpeed;
		
		yaw = referenceBlock.RollIndicator;
		
		double alignedPitch = 	(Math.Cos(angleRoll) * pitch + Math.Sin(angleRoll) * yaw) - mouseInput.X;
		double alignedYaw = 	(-Math.Sin(angleRoll) * pitch + Math.Cos(angleRoll) * yaw) + mouseInput.Y;
		double alignedRoll = roll;
        ApplyGyroOverride(alignedPitch, alignedYaw, alignedRoll, gyros, referenceBlock);

        overrideStatus = $"\n\n           SAFETY OVERRIDE ACTIVE"; //\nYaw : {yawSpeed}";
    }
    else
    {
        foreach (IMyGyro thisGyro in gyros)
        {
            thisGyro.SetValue("Override", false);
        }
		foreach (IMyThrust thruster in thrusters){
			thruster.ThrustOverride = 0.0f;
		}
        overrideStatus = "";
    }
}

Vector3D VectorProjection(Vector3D a, Vector3D b) //proj a on b    
{
    Vector3D projection = a.Dot(b) / b.LengthSquared() * b;
    return projection;
}

int VectorCompareDirection(Vector3D a, Vector3D b) //returns -1 if vectors return negative dot product 
{
    double check = a.Dot(b);
    if (check < 0)
        return -1;
    else
        return 1;
}

double VectorAngleBetween(Vector3D a, Vector3D b) //returns radians 
{
    if (a.LengthSquared() == 0 || b.LengthSquared() == 0)
        return 0;
    else
        return Math.Acos(MathHelper.Clamp(a.Dot(b) / a.Length() / b.Length(), -1, 1));
}

//Whip's ApplyGyroOverride Method v9 - 8/19/17
void ApplyGyroOverride(double pitch_speed, double yaw_speed, double roll_speed, List<IMyGyro> gyro_list, IMyTerminalBlock reference)
{
    var rotationVec = new Vector3D(-pitch_speed, yaw_speed, roll_speed); //because keen does some weird stuff with signs 
    var shipMatrix = reference.WorldMatrix;
    var relativeRotationVec = Vector3D.TransformNormal(rotationVec, shipMatrix);

    foreach (var thisGyro in gyro_list)
    {
        var gyroMatrix = thisGyro.WorldMatrix;
        var transformedRotationVec = Vector3D.TransformNormal(relativeRotationVec, Matrix.Transpose(gyroMatrix));

        thisGyro.Pitch = (float)transformedRotationVec.X;
        thisGyro.Yaw = (float)transformedRotationVec.Y;
        thisGyro.Roll = (float)transformedRotationVec.Z;
        thisGyro.GyroOverride = true;
    }
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