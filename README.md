# Bragi


## Continuous integration (CI)

CI workflows are run prior to pushing or merging code in github to improve and enforce code standards and quality to downstream users.

- `CI-build.yaml`
    - This workflow tries to build the robot code prior to accepting the code in the remote github repository.
    - The build uses the `settings.gradle` options.
    - If the build fails, then the code is not accepted into the remote github repository.
    - To pass a failing build, fix the code until the build is successful locally, then commit, merge and push your changes.

- `CI-format-checker.yaml`
    - This workflow checks the format of the code format for consistency which improves code readability.
    - The build uses `spotless` settings in the `build.gradle` file. Among other settings, the `googleJavaFormat` is used for checking the formatting of `java` code. 
    - Make sure the `.gitattributes` file is present if code is being developed or committed to the remote branch from different OSs (PC and Mac[Mr. Murphy]).
    - If the build fails, then the code is not accepted into the remote github repository.
    - To pass failing code, run the `.\gradlew spotlessApply` command fix the code formatting. Run the `.\gradlew spotlessCheck` command to until the build is successful locally, then commit, merge and push your changes.
    - TL;DR: Run `.\gradlew spotlessApply` in the terminal in the `.\BragiCode` directory prior to pushing code to the remote.