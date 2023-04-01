Import("env")

# Add semihosting feature
env.Append(
    LINKFLAGS=["--specs=rdimon.specs"],
    LIBS=["rdimon"]
)
