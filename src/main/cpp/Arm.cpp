#include "Arm.h"

Arm::Arm(){
	m_armAngle.SetNeutralMode(NeutralMode::Brake);
	m_armExtend.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Arm::Extension(int in, int out){
	if(!m_extensionSwitch.Get() && in && !out){
		m_armExtend.Set(1);
	}else if(out && !in){
		m_armExtend.Set(-1);
	}else{
		m_armExtend.Set(0);
	}
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

void Arm::AutoPosition(int angle, bool extending, bool enabled){
	if(!enabled){
		positionSpeed = .3;
	}else{
		double dist = abs(m_armEncoder.GetDistance());

		if(extending){
			if(dist < angle-2){
				m_armAngle.Set(positionSpeed);
			}else if(dist > (angle+2)){
				m_armAngle.Set(-positionSpeed);
			}else if((dist > (angle-2)) && (dist < (angle+2))){
				positionSpeed = .05;
				m_armAngle.Set(0);
			}
		}else{
			if(!m_extensionSwitch.Get()){
				m_armExtend.Set(1);
			}else{
				m_armExtend.Set(0);
				if(dist < angle-2){
					m_armAngle.Set(positionSpeed);
				}else if(dist > (angle+2)){
					m_armAngle.Set(-positionSpeed);
				}else if((dist > (angle-2)) && (dist < (angle+2))){
					positionSpeed = .05;
					m_armAngle.Set(0);
				}
			}
		}
	}
}