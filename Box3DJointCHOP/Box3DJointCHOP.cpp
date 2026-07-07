#include "Box3DJointCHOP.h"

#include <cctype>
#include <cstring>
#include <string>
#include <vector>

using namespace tdb3;

namespace
{

constexpr float kDegToRad = 0.01745329251994329577f;

constexpr char SolverName[] = "Solver";
constexpr char TypeName[] = "Type";
constexpr char JointsName[] = "Joints";
constexpr char CountName[] = "Count";
constexpr char AxisName[] = "Axis";
constexpr char CollideName[] = "Collide";

// Body pair slots, Constant CHOP style: a Joints count parameter enables the
// first N rows. The C++ SDK has no real sequential (+/-) parameters, so the
// slots are a fixed pool declared up front. Slot 0 keeps the legacy
// unsuffixed names (Bodya/Idxa/Bodyb/Idxb) so existing setups keep their
// values; slots 1.. use Bodya2, Idxa2, ...
constexpr int kMaxPairs = 8;

struct SlotNames
{
	std::string bodya, idxa, bodyb, idxb;
	std::string bodyaLabel, idxaLabel, bodybLabel, idxbLabel;
};

const SlotNames& slotNames( int slot )
{
	static const std::vector<SlotNames> all = [] {
		std::vector<SlotNames> v;
		v.reserve( kMaxPairs );
		for ( int i = 0; i < kMaxPairs; ++i )
		{
			SlotNames n;
			const std::string suffix = i == 0 ? "" : std::to_string( i + 1 );
			const std::string humanIndex = " " + std::to_string( i + 1 );
			n.bodya = "Bodya" + suffix;
			n.idxa = "Idxa" + suffix;
			n.bodyb = "Bodyb" + suffix;
			n.idxb = "Idxb" + suffix;
			n.bodyaLabel = "Body A" + humanIndex;
			n.idxaLabel = "Index A" + humanIndex;
			n.bodybLabel = "Body B" + humanIndex;
			n.idxbLabel = "Index B" + humanIndex;
			v.push_back( n );
		}
		return v;
	}();
	return all[slot];
}

constexpr char HertzName[] = "Hertz";
constexpr char DampingName[] = "Damping";
constexpr char AutolengthName[] = "Autolength";
constexpr char LengthName[] = "Length";
constexpr char LimitName[] = "Limit";
constexpr char LowerName[] = "Lower";
constexpr char UpperName[] = "Upper";
constexpr char MinlengthName[] = "Minlength";
constexpr char MaxlengthName[] = "Maxlength";
constexpr char ConelimitName[] = "Conelimit";
constexpr char ConeName[] = "Cone";
constexpr char MotorName[] = "Motor";
constexpr char MotorspeedName[] = "Motorspeed";
constexpr char MaxmotortorqueName[] = "Maxmotortorque";

constexpr int kNumOutputChannels = 7;
constexpr const char* kOutputChannelNames[kNumOutputChannels] = { "ax", "ay", "az", "bx", "by", "bz", "active" };

// Shared joint settings (type, axis, dynamics); the body pair comes from the
// slot parameters per joint.
JointSpec readSharedSpec( const OP_Inputs* inputs )
{
	JointSpec spec;
	spec.type = inputs->getParInt( TypeName );

	double x, y, z;
	inputs->getParDouble3( AxisName, x, y, z );
	spec.axisX = (float)x;
	spec.axisY = (float)y;
	spec.axisZ = (float)z;

	spec.hertz = (float)inputs->getParDouble( HertzName );
	spec.dampingRatio = (float)inputs->getParDouble( DampingName );

	bool autoLength = inputs->getParInt( AutolengthName ) != 0;
	spec.length = autoLength ? -1.0f : (float)inputs->getParDouble( LengthName );

	if ( inputs->getParInt( LimitName ) != 0 )
	{
		spec.enableLimit = true;
		spec.lowerAngle = (float)inputs->getParDouble( LowerName ) * kDegToRad;
		spec.upperAngle = (float)inputs->getParDouble( UpperName ) * kDegToRad;
		spec.minLength = (float)inputs->getParDouble( MinlengthName );
		spec.maxLength = (float)inputs->getParDouble( MaxlengthName );
	}

	if ( inputs->getParInt( ConelimitName ) != 0 )
	{
		spec.enableConeLimit = true;
		spec.coneAngle = (float)inputs->getParDouble( ConeName ) * kDegToRad;
	}

	if ( inputs->getParInt( MotorName ) != 0 )
	{
		spec.enableMotor = true;
		double speed = inputs->getParDouble( MotorspeedName );
		spec.motorSpeed = spec.type == 2 ? (float)speed * kDegToRad : (float)speed;
		spec.maxMotorForce = (float)inputs->getParDouble( MaxmotortorqueName );
	}

	spec.collideConnected = inputs->getParInt( CollideName ) != 0;
	return spec;
}

int readPairCount( const OP_Inputs* inputs )
{
	int pairs = inputs->getParInt( JointsName );
	if ( pairs < 1 )
	{
		pairs = 1;
	}
	if ( pairs > kMaxPairs )
	{
		pairs = kMaxPairs;
	}
	return pairs;
}

int readSeriesCount( const OP_Inputs* inputs )
{
	int count = inputs->getParInt( CountName );
	return count < 1 ? 1 : count;
}

} // namespace

extern "C"
{

DLLEXPORT
void FillCHOPPluginInfo( CHOP_PluginInfo* info )
{
	info->apiVersion = CHOPCPlusPlusAPIVersion;

	OP_CustomOPInfo& customInfo = info->customOPInfo;
	customInfo.opType->setString( "Box3djointchop" );
	customInfo.opLabel->setString( "Box3D Joint CHOP" );
	customInfo.opIcon->setString( "JCH" );
	customInfo.authorName->setString( "lolo" );
	customInfo.authorEmail->setString( "xxxlolx@gmail.com" );

	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
	// Kick-start cookEveryFrame on scene load: a Joint CHOP is usually not
	// wired to anything, and without this it would never cook (and never
	// register its joints) until someone opened its viewer.
	customInfo.cookOnStart = true;
}

DLLEXPORT
CHOP_CPlusPlusBase* CreateCHOPInstance( const OP_NodeInfo* info )
{
	return new Box3DJointCHOP( info );
}

DLLEXPORT
void DestroyCHOPInstance( CHOP_CPlusPlusBase* instance )
{
	delete (Box3DJointCHOP*)instance;
}

} // extern "C"

Box3DJointCHOP::Box3DJointCHOP( const OP_NodeInfo* info ) : myOpId( info->opId )
{
}

Box3DJointCHOP::~Box3DJointCHOP()
{
	unregisterJoint();
}

void Box3DJointCHOP::unregisterJoint()
{
	if ( myJointRegistered && mySolverOpId != 0 )
	{
		SolverCore* core = Registry::find( mySolverOpId );
		if ( core != nullptr )
		{
			core->removeJointNode( myOpId );
		}
	}
	myJointRegistered = false;
}

void Box3DJointCHOP::getGeneralInfo( CHOP_GeneralInfo* ginfo, const OP_Inputs*, void* )
{
	// Cook unconditionally: joints exist because this node cooks — nothing
	// consumes its output channels in a typical setup.
	ginfo->cookEveryFrame = true;
	ginfo->timeslice = false;
}

bool Box3DJointCHOP::getOutputInfo( CHOP_OutputInfo* info, const OP_Inputs* inputs, void* )
{
	// One sample per joint: pair slots x series count.
	info->numChannels = kNumOutputChannels;
	info->numSamples = readPairCount( inputs ) * readSeriesCount( inputs );
	info->startIndex = 0;
	return true;
}

void Box3DJointCHOP::getChannelName( int32_t index, OP_String* name, const OP_Inputs*, void* )
{
	name->setString( kOutputChannelNames[index] );
}

void Box3DJointCHOP::execute( CHOP_Output* output, const OP_Inputs* inputs, void* )
{
	myWarning.clear();

	const OP_CHOPInput* solverInput = inputs->getParCHOP( SolverName );
	SolverCore* core = solverInput != nullptr ? Registry::find( solverInput->opId ) : nullptr;

	int type = inputs->getParInt( TypeName );
	inputs->enablePar( AutolengthName, type == 0 );
	inputs->enablePar( LengthName, type == 0 && inputs->getParInt( AutolengthName ) == 0 );
	inputs->enablePar( MinlengthName, type == 0 );
	inputs->enablePar( MaxlengthName, type == 0 );
	inputs->enablePar( LowerName, type == 1 || type == 2 );
	inputs->enablePar( UpperName, type == 1 || type == 2 );
	inputs->enablePar( ConelimitName, type == 1 );
	inputs->enablePar( ConeName, type == 1 );
	inputs->enablePar( MotorName, type == 0 || type == 2 );
	inputs->enablePar( MotorspeedName, type == 0 || type == 2 );
	inputs->enablePar( MaxmotortorqueName, type == 0 || type == 2 );
	inputs->enablePar( AxisName, type == 1 || type == 2 );

	const int pairCount = readPairCount( inputs );
	const int seriesCount = readSeriesCount( inputs );
	for ( int s = 0; s < kMaxPairs; ++s )
	{
		const SlotNames& names = slotNames( s );
		bool on = s < pairCount;
		inputs->enablePar( names.bodya.c_str(), on );
		inputs->enablePar( names.idxa.c_str(), on );
		inputs->enablePar( names.bodyb.c_str(), on );
		inputs->enablePar( names.idxb.c_str(), on );
	}

	const int totalJoints = pairCount * seriesCount;
	myStates.assign( (size_t)totalJoints, JointState() );
	myActive = false;

	if ( core == nullptr )
	{
		myWarning = solverInput == nullptr ? "Set the Solver parameter to a Box3D Solver CHOP."
											   : "The Solver parameter does not point to a Box3D Solver CHOP.";
		unregisterJoint();
		mySolverOpId = 0;
	}
	else
	{
		uint32_t solverOpId = solverInput->opId;
		if ( solverOpId != mySolverOpId )
		{
			unregisterJoint();
			mySolverOpId = solverOpId;
		}

		JointSpec shared = readSharedSpec( inputs );

		std::vector<JointSpec> specs;
		specs.reserve( (size_t)totalJoints );
		bool anyBodyA = false;
		for ( int s = 0; s < pairCount; ++s )
		{
			const SlotNames& names = slotNames( s );
			const char* bodyA = inputs->getParString( names.bodya.c_str() );
			const char* bodyB = inputs->getParString( names.bodyb.c_str() );

			JointSpec base = shared;
			base.bodyA = bodyA != nullptr ? bodyA : "";
			base.indexA = inputs->getParInt( names.idxa.c_str() );
			base.bodyB = bodyB != nullptr ? bodyB : "";
			base.indexB = inputs->getParInt( names.idxb.c_str() );
			anyBodyA = anyBodyA || !base.bodyA.empty();

			// Count > 1 turns each pair into a series with incrementing
			// indices (chains over an Instances CHOP group).
			for ( int c = 0; c < seriesCount; ++c )
			{
				JointSpec si = base;
				si.indexA = base.indexA + c;
				if ( !si.bodyB.empty() )
				{
					si.indexB = base.indexB + c;
				}
				specs.push_back( si );
			}
		}

		if ( !anyBodyA )
		{
			myWarning = "Set Body A 1 to a Box3D Body SOP or Box3D Instances CHOP (path or node name).";
		}

		core->setJointNodeList( myOpId, specs );
		myJointRegistered = true;

		for ( int i = 0; i < totalJoints; ++i )
		{
			JointState& st = myStates[i];
			st.active = core->getJointAnchors( myOpId, i, st.a, st.b );
			myActive = myActive || st.active;
		}

		if ( !myActive && myWarning.empty() )
		{
			myWarning = "Joint not resolved yet (check Body A/B paths and indices, and ensure those body nodes are cooking).";
		}
	}

	// One sample per joint; TD may have sized the output on an older cook, so
	// clamp defensively.
	int samples = output->numSamples < totalJoints ? output->numSamples : totalJoints;
	for ( int i = 0; i < samples; ++i )
	{
		const JointState& st = myStates[i];
		output->channels[0][i] = st.a[0];
		output->channels[1][i] = st.a[1];
		output->channels[2][i] = st.a[2];
		output->channels[3][i] = st.b[0];
		output->channels[4][i] = st.b[1];
		output->channels[5][i] = st.b[2];
		output->channels[6][i] = st.active ? 1.0f : 0.0f;
	}
	for ( int i = samples; i < output->numSamples; ++i )
	{
		for ( int ch = 0; ch < kNumOutputChannels; ++ch )
		{
			output->channels[ch][i] = 0.0f;
		}
	}
}

int32_t Box3DJointCHOP::getNumInfoCHOPChans( void* )
{
	return kNumOutputChannels;
}

void Box3DJointCHOP::getInfoCHOPChan( int32_t index, OP_InfoCHOPChan* chan, void* )
{
	// First joint's anchors (the full per-joint set is on the CHOP output,
	// one sample per joint).
	chan->name->setString( kOutputChannelNames[index] );
	static const JointState kEmpty;
	const JointState& st = myStates.empty() ? kEmpty : myStates[0];
	if ( index < 3 )
	{
		chan->value = st.a[index];
	}
	else if ( index < 6 )
	{
		chan->value = st.b[index - 3];
	}
	else
	{
		chan->value = st.active ? 1.0f : 0.0f;
	}
}

void Box3DJointCHOP::getWarningString( OP_String* warning, void* )
{
	if ( !myWarning.empty() )
	{
		warning->setString( myWarning.c_str() );
	}
}

void Box3DJointCHOP::setupParameters( OP_ParameterManager* manager, void* )
{
	{
		OP_StringParameter p;
		p.name = SolverName;
		p.label = "Solver";
		p.page = "Joint";
		// Auto-bind: a sibling solver with TD's default name resolves on
		// creation (paths are relative to this node).
		p.defaultValue = "box3dsolver1";
		manager->appendCHOP( p );
	}

	{
		OP_StringParameter p;
		p.name = TypeName;
		p.label = "Type";
		p.page = "Joint";
		p.defaultValue = "distance";
		const char* names[] = { "distance", "spherical", "revolute", "weld" };
		const char* labels[] = { "Distance", "Spherical", "Revolute", "Weld" };
		manager->appendMenu( p, 4, names, labels );
	}

	{
		OP_NumericParameter p;
		p.name = JointsName;
		p.label = "Joints";
		p.page = "Joint";
		p.defaultValues[0] = 1;
		p.minValues[0] = 1;
		p.maxValues[0] = kMaxPairs;
		p.minSliders[0] = 1;
		p.maxSliders[0] = kMaxPairs;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		manager->appendInt( p );
	}

	{
		OP_NumericParameter p;
		p.name = CountName;
		p.label = "Count (per pair)";
		p.page = "Joint";
		p.defaultValues[0] = 1;
		p.minValues[0] = 1;
		p.minSliders[0] = 1;
		p.maxSliders[0] = 128;
		p.clampMins[0] = true;
		manager->appendInt( p );
	}

	// Body pair slots; the Joints parameter enables the first N.
	for ( int s = 0; s < kMaxPairs; ++s )
	{
		const SlotNames& names = slotNames( s );

		{
			OP_StringParameter p;
			p.name = names.bodya.c_str();
			p.label = names.bodyaLabel.c_str();
			p.page = "Bodies";
			manager->appendString( p );
		}

		{
			OP_NumericParameter p;
			p.name = names.idxa.c_str();
			p.label = names.idxaLabel.c_str();
			p.page = "Bodies";
			p.defaultValues[0] = 0;
			p.minValues[0] = 0;
			p.minSliders[0] = 0;
			p.maxSliders[0] = 100;
			p.clampMins[0] = true;
			manager->appendInt( p );
		}

		{
			OP_StringParameter p;
			p.name = names.bodyb.c_str();
			p.label = names.bodybLabel.c_str();
			p.page = "Bodies";
			manager->appendString( p );
		}

		{
			OP_NumericParameter p;
			p.name = names.idxb.c_str();
			p.label = names.idxbLabel.c_str();
			p.page = "Bodies";
			p.defaultValues[0] = 0;
			p.minValues[0] = 0;
			p.minSliders[0] = 0;
			p.maxSliders[0] = 100;
			p.clampMins[0] = true;
			manager->appendInt( p );
		}
	}

	{
		OP_NumericParameter p;
		p.name = AxisName;
		p.label = "Axis";
		p.page = "Joint";
		p.defaultValues[0] = 0.0;
		p.defaultValues[1] = 0.0;
		p.defaultValues[2] = 1.0;
		for ( int i = 0; i < 3; ++i )
		{
			p.minSliders[i] = -1.0;
			p.maxSliders[i] = 1.0;
		}
		manager->appendXYZ( p );
	}

	{
		OP_NumericParameter p;
		p.name = CollideName;
		p.label = "Connected Bodies Collide";
		p.page = "Joint";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = HertzName;
		p.label = "Spring Hertz";
		p.page = "Dynamics";
		p.defaultValues[0] = 4.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 20.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = DampingName;
		p.label = "Spring Damping";
		p.page = "Dynamics";
		p.defaultValues[0] = 0.7;
		p.minValues[0] = 0.0;
		p.maxValues[0] = 10.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 5.0;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = AutolengthName;
		p.label = "Auto Length (Distance)";
		p.page = "Dynamics";
		p.defaultValues[0] = 1.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = LengthName;
		p.label = "Length";
		p.page = "Dynamics";
		p.defaultValues[0] = 1.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 10.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = LimitName;
		p.label = "Enable Limit";
		p.page = "Dynamics";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = LowerName;
		p.label = "Lower Angle (deg)";
		p.page = "Dynamics";
		p.defaultValues[0] = -45.0;
		p.minSliders[0] = -180.0;
		p.maxSliders[0] = 180.0;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = UpperName;
		p.label = "Upper Angle (deg)";
		p.page = "Dynamics";
		p.defaultValues[0] = 45.0;
		p.minSliders[0] = -180.0;
		p.maxSliders[0] = 180.0;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = MinlengthName;
		p.label = "Min Length";
		p.page = "Dynamics";
		p.defaultValues[0] = 0.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 10.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = MaxlengthName;
		p.label = "Max Length";
		p.page = "Dynamics";
		p.defaultValues[0] = 1.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 10.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = ConelimitName;
		p.label = "Enable Cone Limit";
		p.page = "Dynamics";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = ConeName;
		p.label = "Cone Angle (deg)";
		p.page = "Dynamics";
		p.defaultValues[0] = 45.0;
		p.minValues[0] = 0.0;
		p.maxValues[0] = 179.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 120.0;
		p.clampMins[0] = true;
		p.clampMaxes[0] = true;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = MotorName;
		p.label = "Enable Motor";
		p.page = "Dynamics";
		p.defaultValues[0] = 0.0;
		manager->appendToggle( p );
	}

	{
		OP_NumericParameter p;
		p.name = MotorspeedName;
		p.label = "Motor Speed (deg/s | units/s)";
		p.page = "Dynamics";
		p.defaultValues[0] = 0.0;
		p.minSliders[0] = -360.0;
		p.maxSliders[0] = 360.0;
		manager->appendFloat( p );
	}

	{
		OP_NumericParameter p;
		p.name = MaxmotortorqueName;
		p.label = "Max Motor Torque / Force";
		p.page = "Dynamics";
		p.defaultValues[0] = 0.0;
		p.minValues[0] = 0.0;
		p.minSliders[0] = 0.0;
		p.maxSliders[0] = 10000.0;
		p.clampMins[0] = true;
		manager->appendFloat( p );
	}
}
