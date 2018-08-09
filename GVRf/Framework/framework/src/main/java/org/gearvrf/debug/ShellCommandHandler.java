/* Copyright 2016 Samsung Electronics Co., LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.gearvrf.debug;

import java.util.List;

import javax.script.ScriptEngine;

import org.gearvrf.GVRContext;
import org.gearvrf.GVRVersion;
import org.gearvrf.debug.cli.Command;
import org.gearvrf.debug.cli.HelpCommandHandler;
import org.gearvrf.debug.cli.Shell;
import org.gearvrf.debug.cli.ShellDependent;
import org.gearvrf.script.IScriptManager;

/**
 * Shell commands for GVRf debug console.
 */
public class ShellCommandHandler implements ShellDependent {
    protected Shell mShell;
    protected GVRContext mGVRContext;
    protected HelpCommandHandler mHelpHandler = new HelpCommandHandler();

    protected ScriptHandler mScriptHandler;
    protected List<String> mSavedPath;

    public ShellCommandHandler(GVRContext gvrContext) {
        mGVRContext = gvrContext;
    }

    @Command
    public String js() {
        return enterLanguage(IScriptManager.LANG_JAVASCRIPT);
    }

    private String enterLanguage(String language) {
        IScriptManager sm = mGVRContext.getScriptManager();
        if (sm == null) {
            return "Script extension is not enabled";
        }

        ScriptEngine engine = sm.getEngine(language);
        if (engine == null) {
            return "Cannot find the language engine for " + language;
        }

        mScriptHandler = new ScriptHandler(mGVRContext, language, engine);
        mShell.setLineProcessor(mScriptHandler);

        return null;
    }

    @Command
    public String version() {
        return GVRVersion.CURRENT;
    }

    @Command
    public Object help() {
        return mHelpHandler.help();
    }

    @Command
    public void exit() {
        // empty
    }

    @Override
    public void cliSetShell(Shell theShell) {
        mShell = theShell;
        mHelpHandler.cliSetShell(mShell);
    }
}
