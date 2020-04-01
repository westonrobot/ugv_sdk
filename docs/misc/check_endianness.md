On a Big Endian-System (Solaris on SPARC)

```
$ echo -n I | od -to2 | head -n1 | cut -f2 -d" " | cut -c6 
0
```
On a little endian system (Linux on x86)
```
$ echo -n I | od -to2 | head -n1 | cut -f2 -d" " | cut -c6 
1
```
The solution above is clever and works great for Linux *86 and Solaris Sparc.

I needed a shell-only (no Perl) solution that also worked on AIX/Power and HPUX/Itanium. Unfortunately the last two don't play nice: AIX reports "6" and HPUX gives an empty line.

Using your solution, I was able to craft something that worked on all these Unix systems:
```
$ echo I | tr -d [:space:] | od -to2 | head -n1 | awk '{print $2}' | cut -c6
```
Regarding the Python solution someone posted, it does not work in Jython because the JVM treats everything as Big. If anyone can get it to work in Jython, please post!

Also, I found this, which explains the endianness of various platforms. Some hardware can operate in either mode depending on what the O/S selects: http://labs.hoffmanlabs.com/node/544

If you're going to use awk this line can be simplified to:
```
echo -n I | od -to2 | awk '{ print substr($2,6,1); exit}'
```
For small Linux boxes that don't have 'od' (say OpenWrt) then try 'hexdump':
```
echo -n I | hexdump -o | awk '{ print substr($2,6,1); exit}'
```

Reference: 

* [1] https://serverfault.com/questions/163487/how-to-tell-if-a-linux-system-is-big-endian-or-little-endian
* [2] https://wiki.rdu.im/_pages/Knowledge-Base/Computing/Computing.html