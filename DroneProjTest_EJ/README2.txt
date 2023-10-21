The DroneClasses folder must be put in "~\Documents\Arduino\libraries" so that the include statements work. 


It is important that the classes be created in separate files because the C++ compiler works top to bottom. So if we don't declare all the classes at the top of the main.ino file, we can not declare or use our custom classes (PID, Integrator...) until our class declaration. Which would result in messy and/or unusable code. 