package path_solver

import "boxbot-go/pkg/ik_solver"

type SimplePath struct {
	Duration                            float64
	Target                              ik_solver.JointState
	ArmMap, BaseMap, ServoMap, TowerMap *VelMap
}
