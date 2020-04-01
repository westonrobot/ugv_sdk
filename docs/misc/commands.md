# Commands

## Create gif animation using "imagemagick"

```
$ convert -delay 120 -loop 0 *.png animated.gif
```

## Get statistics of the code base

```
$ sudo apt install cloc
$ cd ~/Workspace/librav/src
$ cloc --exclude-dir=cmake,lcmtypes,third_party .
```

## Create a pair of VSP’s

```
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

```
$ cat nmea_test.txt > /dev/pts/6
```

## git subtree

Adding the sub-project as a remote
```
$ git remote add -f [remote-name] [remote-url]
$ git subtree add --prefix [sub-project-name] [remote-name] [branch-name] --squash
```

Update the sub-project
```
$ git fetch [remote-name] [branch-name] 
$ git subtree pull --prefix [sub-project-name] [remote-name] [branch-name] --squash
```

Push to remote
```
$ git subtree push --prefix=[sub-project-name] [remote-name] [branch-name] 
```

Firmware branch update
```
$ git fetch fw_origin pios_pixcar
$ git subtree pull --prefix firmware fw_origin pios_pixcar --squash
```

## Reference

* [Git subtree: the alternative to Git submodule](https://www.atlassian.com/blog/git/alternatives-to-git-submodule-git-subtree)
* [Virtual serial port: socat](https://justcheckingonall.wordpress.com/2009/06/09/howto-vsp-socat/)