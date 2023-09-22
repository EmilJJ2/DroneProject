# DroneProject

## Github Instructions

- Install Git
    - https://git-scm.com/downloads
- Send me your Github Email
  - I need to add you as a contributor so that you can make changes
- Authorize Git
  - https://docs.github.com/en/get-started/getting-started-with-git/setting-your-username-in-git
    - Setting username for Git
    - $ git config --global user.name "[YOUR NAME]"
  - https://docs.github.com/en/account-and-profile/setting-up-and-managing-your-personal-account-on-github/managing-email-preferences/setting-your-commit-email-address
    - Setting commit email
    - $ git config --global user.email "[YOUR GITHUB EMAIL]"
    -   Very important that the email used here is the same as the one used for your GitHub account
- Clone repository
    - Navigate to directory in Git Bash where you want the repository to be cloned into
        - $ cd [FOLDER NAME]
            - cd by itself goes back to the parent directory
    - $ git clone [REPO URL]
- Make a new branch with your name
    - $ git branch [NAME]
    - Switch to the branch with
        - $ git switch [NAME]
- Push back to main branch
    - $ git push origin [BRANCH NAME]
    - Login when prompted

GENERAL INFO
- To pull from other branches into yours use:
  - $ git pull origin [THE BRANCH YOU WANT INFO FROM]
- To publish changes you have made
  - $ git add --all
    - This stages all changes to commit. To only stage some changes look at Github's website
  - $ git commit -m "[WRITE A MESSAGE HERE EXPLAINING YOUR CHANGES]"
  - $ git push origin [BRANCH NAME]

**READ THIS!!**
It is VERY important that you only commit and push changes from your own branch and not others. If you are confused about anything here, ask me or wait until the next meeting and we can figure it out. If you see somewhere I wrote something wrong please let me know and I will change it. THANKS!!
