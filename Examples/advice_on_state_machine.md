Another thing to note is that in a state machine, the next state is found using the first function name in the list.  So, if you have a state machine that looks like:

```
move0=StateMachine(
    ([north,until_xy(3,27),turn_purple,off],"east"),
    ([east,until_xy(10,27),turn_red,off],"south"),
    ([south,until_xy(10,21.5),turn_purple,off],"_end_simulation"),
)
```

the `"east"` next state points to the next action starting with `[east,until_xy...]`

Because of this, the order doesn't matter (other than the very first one, which is the first state), which makes this an equivalent state machine:

```
move0=StateMachine(
    ([north,until_xy(3,27),turn_purple,off],"east"),
    ([south,until_xy(10,21.5),turn_purple,off],"_end_simulation"),
    ([east,until_xy(10,27),turn_red,off],"south"),
)
```

Because of this, you can't have two lists of actions start with the same name, like:

```
move0=StateMachine(
    ([north,until_xy(3,27),turn_purple,off],"east"),
    ([east,until_xy(10,27),turn_red,off],"south"),
    ([south,until_xy(10,21.5),turn_purple,off],"east"),
    ([east,until_xy(10,50),turn_red,off],"_end_simulation"),
)
```

because it wouldn't know which `east` to go to.  in those cases, the easiest thing is to define another east, like:

```
def east2(t,act):
    robot['disk0'].F=10
    return True
```

and use that one in the state machine:

```
move0=StateMachine(
    ([north,until_xy(3,27),turn_purple,off],"east"),
    ([east,until_xy(10,27),turn_red,off],"south"),
    ([south,until_xy(10,21.5),turn_purple,off],"east"),
    ([east,until_xy(10,50),turn_red,off],"_end_simulation"),
)
```

