# Bragi


## Continuous integration (CI)

Two CI workflows are run prior to pushing or merging code in github to improve and enforce code quality to downstream users.

- `CI-build.yaml`
    - This workflow tries to build the robot code prior to accepting the code in the remote github repository.
    - The build uses the `settings.gradle` options.
    - If the build fails then the code is not accepted into the remote github repository.
    - To pass, fix the code until the build successful locally, then commit or merge and push your changes.

- `CI-format-checker`
    - This workflow checks the format of the code for consistency which improves code readability.
    - The build uses `spotless` settings in the `build.gradle` file. Among other settings, the `googleJavaFormat` is used for checking the formatting of `java` code. 
    - Make sure the `.gitattributes` file is present if code is being developed or committed from different OSs (PC and Mac[Mr. Murphy])
    - If the build fails then the code is not accepted into the remote github repository.
    - To pass, run the `.\gradlew spotlessApply` command fix the code formatting. Run the `.\gradlew spotlessCheck` command to until the build successful locally, then commit or merge and push your changes.
    
    #TODO: Set up `pre-commit` or `git` hooks to autoformat using `spotless` on save.