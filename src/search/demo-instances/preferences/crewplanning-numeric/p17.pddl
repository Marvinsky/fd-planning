(define (problem CrewPlanning_1crew_2day)
(:domain CrewPlanning)
(:objects
	d1 d2 d3 - Day

	c1 - CrewMember
	mcs1 - MedicalState

	spaceshipFilter - FilterState

	pa1 pa2 pa3 pa4 pa5 pa6 pa7 pa8 pa9 pa10 - PayloadAct
)
(:init
	(= (total-cost) 0)
	(currentday d1)
	(next d1 d2)
	(next d2 d3)

	(active c1 d1)
	(= (available_time c1 d1) 1440)
	(active c1 d2)
	(= (available_time c1 d2) 1440)

	(= (crew_efficiency c1 d1) 9)
	(= (crew_efficiency c1 d2) 10)

	(= (achieve_time_discount d1) 0)
	(= (achieve_time_discount d2) 10)

	(= (payloadact_length pa1) 80)
	(= (payloadact_length pa2) 130)
	(= (payloadact_length pa3) 150)
	(= (payloadact_length pa4) 160)
	(= (payloadact_length pa5) 110)
	(= (payloadact_length pa6) 150)
	(= (payloadact_length pa7) 80)
	(= (payloadact_length pa8) 110)
	(= (payloadact_length pa9) 150)
	(= (payloadact_length pa10) 120)

)

(:goal
(and
	(preference dr (currentday d3))
	(preference mcs1_2 (mcs_finished mcs1 d2))

	(preference filter2 (changed spaceshipFilter d2))


	(preference pref_pa1 (payload_act_completed pa1))
	(preference pref_pa2 (payload_act_completed pa2))
	(preference pref_pa3 (payload_act_completed pa3))
	(preference pref_pa4 (payload_act_completed pa4))
	(preference pref_pa5 (payload_act_completed pa5))
	(preference pref_pa6 (payload_act_completed pa6))
	(preference pref_pa7 (payload_act_completed pa7))
	(preference pref_pa8 (payload_act_completed pa8))
	(preference pref_pa9 (payload_act_completed pa9))
	(preference pref_pa10 (payload_act_completed pa10))
)
)
(:metric maximize
	( - 7685
	(+ (total-cost)
	(* (is-violated dr) 5490)
	    (* (is-violated mcs1_2) 180)
	    (* (is-violated filter2) 180)
	    (* (is-violated pref_pa1) 160)
	    (* (is-violated pref_pa2) 260)
	    (* (is-violated pref_pa3) 150)
	    (* (is-violated pref_pa4) 320)
	    (* (is-violated pref_pa5) 110)
	    (* (is-violated pref_pa6) 225)
	    (* (is-violated pref_pa7) 80)
	    (* (is-violated pref_pa8) 110)
	    (* (is-violated pref_pa9) 300)
	    (* (is-violated pref_pa10) 120)
)))
)
