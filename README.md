Commonly Used Optimization Classes

Date:       11/18/2019
Author:     K. Loux
Copyright:  Kerry Loux 2019
Licence:    MIT (see LICENSE file)

This collection of classes are used in several of my projects, so I decided to break them out as a submodule for ease of maintenance.  This file contains some notes about using the classes contained in this repository.

There are several guides online for using git submodules.  Here is my summary of the most useful git commands.
- To add utilities to your project as a submodule:
$ cd <root directory of your superproject>
$ git submodule add https://github.com/KerryL/optimization.git

NOTE:  To add a submodule within another directory, the destination must be specified following the repository url, so instead of the last step above, it would be:
$ git submodule add https://github.com/KerryL/optimization.git <desired path>/optimization

- Cloning a repository using a submodule now requires a couple of extra steps:
$ git clone ...
$ cd <project directory created by above clone command>
$ git submodule init
$ git submodule update

NOTE:  If your submodules are not in your projects root folder, you'll need to specify that the submodule update should recurse into other folders.  Replace the last command above with:
$ git submodule update --recursive
