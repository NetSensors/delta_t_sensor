Import("env", "projenv")

def after_upload(source, target, env):
    print("after_upload")
    # do some actions

env.AddPostAction("upload", after_upload)

