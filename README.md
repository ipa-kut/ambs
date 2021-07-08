# ambs
Automated Modular Benchamrking System - Successor to the ARAIG Test Stack


## Build Tips

In QTCreator, from the left pane select "Projects".   
Edit your build kit settings. By default this should be using "Desktop" kit, seen in the left view. Select "Build" from there.   
Under "Build Settings" in the main view, expand "Build Steps".   
Next to "CMake Arguments" add the following text: ` --make-args roslint`   
Henceforth all regular builds from QTCreator will also run roslint.   
