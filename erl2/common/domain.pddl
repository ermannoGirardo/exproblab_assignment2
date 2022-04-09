(define (domain cluedo_domain)

    (:requirements
        :strips
        :typing
        :durative-actions 
        :fluents 
        :duration-inequalities
        :universal-preconditions
    )

    (:types
       waypoint
    )
    
    (:predicates
		(robot_at ?wp - waypoint)
		(hint_obtained ?wp - waypoint)
		(hypothesis_checked) ;check consistency of the hp
		(not_initialized)
		(hypothesis_tested)  ;check the truthfulness of the hp
    )	
	
	
	;Durative action to initialize the system
	(:durative-action initialization
		:parameters (?wp - waypoint)
		:duration ( = ?duration 5)
		:condition (over all(not_initialized)) 
		:effect (and
			(at end (not(not_initialized)))
			(at end (robot_at ?wp)))
	)
	
	; Durative action to move to waypoint

	(:durative-action go_to_waypoint
		:parameters (?from ?to - waypoint)
		:duration ( = ?duration 5)
		:condition (and
			(at start (robot_at ?from))
		)
		:effect (and
			(at end (robot_at ?to))
			(at start (not(robot_at ?from)))
		)
	)
	
	; Take hint from the marker once the robot is arrived in the wp
	(:durative-action take_hint
		:parameters (?wp - waypoint)
		:duration ( = ?duration 5)
		:condition (and
			(at start(robot_at ?wp))
		)
		:effect (at end (hint_obtained ?wp))
	)


	;Check the consistency of the hypothesis with armor
	(:durative-action check_hypothesis
		:parameters ()
		:duration ( =?duration 1)
		:condition ( and
			(at start (forall(?wp - waypoint) (hint_obtained ?wp)))
		)
		:effect (at end (hypothesis_checked))
	)	
	
	;Test truthfulness of the hp
	(:durative-action test_hypothesis
		:parameters ()
		:duration ( =?duration 1)
		:condition ( and
			(at start (hypothesis_checked))
		)
		:effect (at end (hypothesis_tested))
	)
)
