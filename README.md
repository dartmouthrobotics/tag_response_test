Tests the response time of the tag sensing component of the system.

One node publishes an image, and records the time stamp of the time it was sent.

The other node subscribes and records the time the "seq comes in"

We can store the time an image was published as the "stamp" value, so that would simplify this down to not needing two files.

So, one node publishes to somewhere stag can see it and then the analysis node subscribes to it and writes out a csv file / produces a plot.
