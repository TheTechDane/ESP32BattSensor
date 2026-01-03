// Logging (from Serial.print statements) can be customised by setting the verbosity here:
//Inspired from : https://github.com/RalphBacon/224-Superior-Serial.print-statements/blob/main/Advanced_Example/log.h
/*  
    E - errors that you really, really should see
    W - warnings that help you find out what's going wrong
    I - Informational text
    D - debging messages that you only want to see when logging
    V - verbose meaning everything is printed

    To use, just #include this header file in your standard Arduino Code and then
    define the level of logging you want by setting the LOGLEVEL to one of the values
    listed here.

    In your code, instead of using Serial.print and Serial.println statements use
    logE / loglnE, logW / loglnW, I, D, V (as you see the level of logging required)
    and only those messages will appear.

    Example:

    You want Errors, Warnings and General logging output but not those marked as Verbose.
    #define LOGLEVEL LOGLEVEL_DEBUGING
*/

#include <string>

// User picks logging level from this list
#define LOGLEVEL_ERRORS 1
#define LOGLEVEL_WARNINGS 2
#define LOGLEVEL_INFO 3
#define LOGLEVEL_DEBUGING 4
#define LOGLEVEL_VERBOSE 5
#define LOGLEVEL_NONE 0

// We want to log everything! Overide this in your sketch
#ifndef LOGLEVEL
#define LOGLEVEL LOGLEVEL_VERBOSE
#warning Log Level is set to default level. \
To change this, type in your sketch:   \
#define LOGLEVEL followed by your preferred level of logging\
before the #include statement
#endif

void logNothing(...)
{
    // This does nothing and will be zapped by the compiler
}

// By default we want a trace stamp output in the first instance
bool traceStampRequired = true;

std::string logStr("");

// The tracestamp looks like [D][mainfunction:45]
#define traceStamp(x, y, z)                            \
    if (traceStampRequired)                            \
    {                                                  \
        Serial.print("[");                             \
        Serial.print(x);                               \
        Serial.print("]");                             \
        Serial.print("[");                             \
        Serial.print(__FUNCTION__);                    \
        Serial.print(":");                             \
        Serial.print(__LINE__);                        \
        Serial.print("] ");                            \
    }                                                  \
    logStr = y;                                      \
    if (z || logStr.find("\n") != std::string::npos) \
        traceStampRequired = true;                     \
    else                                               \
        traceStampRequired = false;

// This is a macro that turns simple (one string) Serial.println statements on and off
#if LOGLEVEL > 0
#define logE(x)              \
    traceStamp("E", x, false); \
    Serial.print(x);
#define loglnE(x)           \
    traceStamp("E", x, true); \
    Serial.println(x)
#else
#define logE(x)   // Nothing to see here
#define loglnE(x) // Or here
#endif

#if LOGLEVEL > 1
#define logW(x)              \
    traceStamp("W", x, false); \
    Serial.print(x);
#define loglnW(x)           \
    traceStamp("W", x, true); \
    Serial.println(x)
#else
#define logW(x)   // Nothing to see here
#define loglnW(x) // Or here
#endif

#if LOGLEVEL > 2
#define logI(x)              \
    traceStamp("I", x, false); \
    Serial.print(x);
#define loglnI(x)           \
    traceStamp("I", x, true); \
    Serial.println(x)
#else
#define logI(x)   /* Nothing to see here */
#define loglnI(x) // Or here
#endif

#if LOGLEVEL > 3
#define logD(x)              \
    traceStamp("D", x, false); \
    Serial.print(x);
#define loglnD(x)           \
    traceStamp("D", x, true); \
    Serial.println(x)
#else
#define logD(x)   /* Nothing to see here */
#define loglnD(x) // Or here
#endif

#if LOGLEVEL > 4
#define logV(x)              \
    traceStamp("V", x, false); \
    Serial.print(x);
#define loglnV(x)           \
    traceStamp("V", x, true); \
    Serial.println(x);
#else
#define logV(x) logNothing(x);
#define loglnV(x) logNothing(x);
#endif