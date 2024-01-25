# Subsystems Guide
### A tutorial for making a subsystem

---

Subsystems are used to represent hardware mechanisms in the code. Implement ```Subsystem``` when a system needs constantly run (otherwise commands should suffice).

Start by creating a new class and having it implement ```Subsystem```.
```java
public class ExampleSubsystem implements Subsystem {/*...*/}
```

All of the commands of Subsystem are well documented within IntelliJ. You can see these commands by control-clicking into Subsystem. 

Within you constructor, call ```this.register()```, which will make ```perioidic()``` be called every tick. 
```java
public ExampleSubsystem(){
    //...
    this.register()
}
```