#include "Arm.h"

Arm::Arm(){
	m_armAngle.SetNeutralMode(NeutralMode::Brake);
	m_armExtend.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Arm::Extension(int in, int out){
	m_armExtend.Set(m_extensionSwitch.Get() ? -out : in-out);
}

void Arm::Angle(float up, float down){
	bool b_up = (up > 0);
	bool b_down = (down > 0);

	if(b_down && !b_up){
		m_armAngle.Set(.3 * down);
	}else if(!b_down && b_up && !m_angleSwitch.Get()){
		m_armAngle.Set(-.3 * up);
	}else{
		m_armAngle.Set(0);
	}
}

void Arm::Intake(int in, int out){
	m_armIntake.Set(in - out);
}

int Arm::AutoPosition(int angle, bool raising, bool lowering){
	if(m_angleSwitch.Get()){m_armEncoder.Reset();}
	double dist = abs(m_armEncoder.GetRaw()/8);
	
	if(raising){
		if(dist < angle-2){
			m_armAngle.Set(positionSpeed);
			return 0;
		}else if(dist > (angle+2)){
			m_armAngle.Set(-positionSpeed);
			return 0;
		}else if((dist > (angle-2)) && (dist < (angle+2))){
			positionSpeed = .05;
			m_armAngle.Set(0);
			return 1;
		}
	}else if(lowering){
		m_armAngle.Set(m_extensionSwitch.Get() ? (m_angleSwitch.Get() ? 0 : -0.3) : 0);
		return m_angleSwitch.Get();
	}else{
		positionSpeed = .3;
		return 0;
	}
	return 0;
}