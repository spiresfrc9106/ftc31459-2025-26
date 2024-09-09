package org.firstinspires.ftc.teamcode.utils;

public class FiniteStateMachine<State extends Enum, Input>{
     /**
     * Takes as input, (currentState, input) and returns the next state
     */
     public interface Transistor<State, Input> {
        State transist(State currentState, Input input);
    }
    State state;
    Transistor<State, Input> transistor;

    public FiniteStateMachine(State initialState, Transistor<State, Input> transistor){
        state = initialState;
        this.transistor = transistor;
    }
    public void update( Input input){
        state = transistor.transist(state, input);
    }
}

