(define (problem cluedo_problem)
	
	(:domain cluedo_domain)

	(:objects 
		  wp0 - waypoint 
		  wp1 - waypoint 
		  wp2 - waypoint 
		  wp3 - waypoint 
		  wp4 - waypoint 
	)

	(:init 
	       (robot_at wp0)	
	 )

	(:goal(and
		(hint_obtained wp1)
		(hint_obtained wp2)
		(hint_obtained wp3)
		(hint_obtained wp4)
		;(robot_at wp2)
		(hypothesis_checked)
		)
	)
)
