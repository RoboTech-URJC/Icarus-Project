

# CONTRIBUTING


### fist steps

1. fork the repo using Github fork label
2. clone the repo into your work_space `git clone [your_forked_repo_url]`
3. create your development branch `git checkout -b [branch_name]`
4. now it's time for you to code your solution...


### take test seriusly

- compile your changes `catkin_make`, preferably using `catkin_make -j 1`
- pass style test `catkin_make roslint`

> KEEP IN MIND: your code must pass CI, so only the code that passed will be accepted.

### create a pull request

If everything before is done correctly, you can push your changes to your forked repository:

1. `git add --all`

2. `git commit -m "info_about_your_changes"`, please write something specific and clear in the commit message.

3. `git push`

and the open a pull request following the next style:

- header: `[topic of your changes] + description of what you are solving`
- body: tell us about what you have changed, modified or added to our code.
- add a the number of issue that solve if proceed using: `#(issue)`



We will revise your code as soon as possible.

Thanks for contributing!
