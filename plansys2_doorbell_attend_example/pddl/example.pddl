(define (domain simple)
    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        robot room door sound
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates

        (robot_at ?r - robot ?ro - room)
        (connected ?ro1 ?ro2 - room)
        (door_at ?d - door ?do - room)
        (door_checked ?d - door)
        (sound_listened ?s - sound)

    );; end Predicates ;;;;;;;;;;;;;;;;;;;;
    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:functions

    );; end Functions ;;;;;;;;;;;;;;;;;;;;
    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:durative-action move
        :parameters (?r - robot ?r1 ?r2 - room)
        :duration ( = ?duration 5)
        :condition (and
            (at start(connected ?r1 ?r2))
            (at start(robot_at ?r ?r1))
        )
        :effect (and
            (at start(not(robot_at ?r ?r1)))
            (at end(robot_at ?r ?r2))
        )
    )

    (:durative-action checkdoor
        :parameters (?r - robot ?d - door ?do - room)
        :duration ( = ?duration 5)
        :condition (and
            (at start(door_at ?d ?do))
            (at start(robot_at ?r ?do))
        )
        :effect (at end(door_checked ?d))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;