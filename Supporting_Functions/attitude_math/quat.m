% Quaternion/Vector/Matrix Utilities
% m-file versions -- no compiling necessary
%
%
% QUATERNION OPPERATIONS :
%    qcon         form a quaternion conjugate
%    qdiff        get successive rotations (quaternion diff)
%    qexp         exponential of a quaternion
%    qinv         form the inverse of a quaternion (even if not normal)
%    qlog         (natrural) logarithm of a quaternion
%    qmult        form a quaternion product
%    qnorm        normalize/properize a quaternion
%    qunprop      undo quaternion properization (smooth derivative)
%    qvtran       change a vector frame using a quaternion transformation
%
% CHANGING A TRANSFORMATION REPRESENTATION :
%    q2error      from a desired and actual quaternion to angle errors
%    q2rdt        from a quaternion to right ascension/declination/twist
%    q2t          from a quaternion to a direction cosine matrix
%    q2vec        from a quaternion to a rotation vector and angle
%    rdt2q        from right ascension/declination/twist to a quaternion
%    t2e          from a direction cosine matrix to Euler angles
%    e2t          from Euler angles to a direction cosine matrix
%    t2q          from a direction cosine matrix to a quaternion
%    vec2q        from a rotation vector and angle to a quaternion
%
% CHANGING A POINTING DIRECTION REPRESENTATION :
%    qatt         get quaternion from primary and secondary vectors
%    rade2v       from right ascension/declination to a 3D vector
%    v2rade       from a 3D vector to right ascension/declination
%    vecab2q      get quaternion that rotates a to b
%
% MISCELLANEOUS
%    cart2sphere  get spherical components of a cartesian vector
%    sphere2cart  get cartesian vector from spherical components 
%    tilde        cross product matrix
%    uunit        unitize a set of vectors  
%    usat         return a saturated version of the input
% 
% October 28, 1996  sws
% August  12, 1997  sws
% May     17, 2002  sws octave version
% May     30  2003  sws added a few more
% June    21, 2005    sws m file only version
% v 1.1
% July     9, 2007  sws merge in improvments (from octave v6) 
%                       removed some useless (in matlab) files
% v 1.2 
%  October 27, 2009 sws some updates, removed some more old files 
% v 1.4 November 19, 2009  update a variety of functions
%                          q2error, q2rdt, qdiff, unit, usat, usign 
% v 1.5 September 4, 2010  update qcon
% v 1.6 September 10, 2010 update comments, improve uqvtran
% 
% v 2.0 February 23, 2013  update to allow row/column input, scalars for
%                          functions with multiple inputs
% v2.1  December 27 2015   add more fns, cleanup
%
% v3.0  April 10, 2026  Modernized and vectorized all functions using
%                       Google Gemini 3.1 Pro. Not verified to work with
%                       Octave, but should work with newer versions that
%                       support implicit expansion (supported since Octave
%                       4.2.0)
% 
% samuel.w.sirlin@jpl.nasa.gov
% bradley.w.taylor@jpl.nasa.gov