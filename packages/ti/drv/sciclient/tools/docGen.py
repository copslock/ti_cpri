#!/usr/bin/python
#
# Copyright (c) 2018, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#   ============================================================================
#   @file   docGen.py
#
#   @desc   Script to extract documentation from source headers and
#           generate design document PDF 
#
#   ============================================================================
#   Revision History
#   <Date>            <Author>          <Revision> 
#   ---------------------------------------------------
#   16-Apr-2018     Sachin Purohit      Initial draft.
#
#   ============================================================================
#   Command: python docGen.py --md <Design Doc template (.md)> --out <Output file name> --source <Space separated source header files> --path <Path to additional files>
#   Example for .rst output : python docGen.py --out lld.rst --source ../sciclient.h ../soc/V0/sciclient_pm.h ../soc/V0/sciclient_fmwMsgParams.h -s
#   Use:' python docGen.py -h ' for information about additional parameters
import re
import os
import sys
import string
import argparse
import textwrap
from subprocess import call

DOXYGENSTYLE = 1

autoFilename = "sci_designdoc_LLD.md"

macros,structures,functions = [],[],[]

class Macro:
    def __init__(self):
        self.name = None
        self.value = None
        self.comment = ''
        self.anchor = ''
        self.doxygenName = ''

class Variable:
    def __init__(self):
        name = None
        self.datatype = None
        self.comment = ''

class Structure:
    def __init__(self):
        self.oldName = ''
        self.newName = ''
        self.variables = []
        self.comment = ''

class Function:
    def __init__(self):
        self.name = None
        self.inputArgs = []
        self.outputType = None
        self.comment = ''
        self.returnValue = ''
        self.requirements = []

class Block:
    def __init__(self):
        self.body = None
        self.primaryComment = None
        self.secondaryComment = ''
        #Increment in value of (,) .
        self.paranStatus =  0
        #Max depth reached by parantheses pair '()'
        self.paranDepth = 0
        #Increment in value of {,} .
        self.curlyStatus = 0
        #Max depth reached by curly brackets pair '{}'
        self.curlyDepth = 0
        #Increment in value of [,] .
        self.squareStatus = 0
        #Max depth reached by square brackets pair '()'
        self.squareDepth = 0
        #Increment in value of <,> .
        self.lessThanStatus = 0
        #Max depth reached by lessThan-greaterThan pair '<>'
        self.lessThanDepth = 0


def pre_main():
    parser=argparse.ArgumentParser(
        description='''This script takes C header files and extracts documentation from C-style &/or Doxygen-style comments.The documentation is then appended to the base .md file and PDF is generated. ''',
        epilog='''Note: 
1)Pandoc(version >1.16) should be installed and added to system path before using this script.
3)In source header files,comments starting with '//' are ignored .
             
Usage example for Sciclient:
python docGen.py --md ../docs/Sciclient_designDoc_content.md --out des.pdf --source ../sciclient.h[set\(range\(89,280\)\)] ../soc/V0/sciclient_pm.h ../soc/V0/sciclient_fmwMsgParams.h --path ../docs/ -s -D ../ -x docs tools firmware lib soc/V0

Additional information: https://confluence.itg.ti.com/pages/viewpage.action?pageId=50647972 .
''',formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument('--out','-o', default='', help='[Required] The output filename.Supported formats are \'.rst,.pdf\'')
    parser.add_argument('--source','-f', nargs='*', default=[], help='[Required] Space separated list of source files.Optionally, you can give line numbers for processing as a python set within [].')
    parser.add_argument('--md','-t', default='', help='[Optional] The template for document as markdown file for generation of design PDF.If no template is provided, only the source documentation will be generated.')
    parser.add_argument('--path','-g', default='', help='[Optional] Search path for graphics and input .md file.GRAPHICS_ABSOLUTE_PATH in the input .md and title .tex(if valid) is replaced by this value for providing include path')
    parser.add_argument('-d','--debug', action='store_true',help='[Optional] For debugging ,this will not remove any generated temorary files')
    parser.add_argument('-s','--silent', action='store_true',help='[Optional] Comments are minimized')
    parser.add_argument('--drvpath','-D', default='', help='[Optional] Absolute path for the driver to generate directory structure')
    parser.add_argument('--exclude','-x', nargs='*', default=[], help='[Optional] Sub directories relative to drvPath ignored while generating the directory structure')
    parser.add_argument('--dump', action='store_true',help='[Optional] Dump macros,structures and functions using pickle ')
    args=parser.parse_args()

    #print parser.parse_args()
    if (args.md == None) | (args.md == ''):
        print "\nNo template is given.Generating only source documentation...\n"
    elif (os.path.isfile(args.md) == False):
        print "\nFile:"+args.md+" doesn't exist.Aborting...\n"
        return
    if (args.out == None) | (args.out == ''):
        print "\nGive proper output filename.\n"
        return
    if len(args.source) == 0:
        print "\nGive atleast one source file.\n"
        return

    main(args)
    
    return

def main(args):

    content = ''
    for fileToBeRead in args.source :
        validLineNos = set()
        if(len(fileToBeRead.split('[')) == 2):
            validLineNos = eval(fileToBeRead.split('[')[1].split(']')[0])
            fileToBeRead = fileToBeRead.split('[')[0]

        if not os.path.isfile(fileToBeRead):
            print fileToBeRead+": File not found!\n"
            return

        numLines = sum(1 for line in open(fileToBeRead))
        if len(validLineNos) == 0:
            validLineNos = set(range(0,numLines))
          
        fr = open(fileToBeRead,'r')
        lines = fr.readlines()
        validLines = []
        for i in range(len(lines)):
            if i in validLineNos:
                validLines.append(lines[i])
        content = content + "".join(validLines)
        fr.close()

    blocks = preprocessing(content,DOXYGENSTYLE)
    blocks = processBlocks(blocks)
    if(DOXYGENSTYLE):
        blocks = doxygenCleanup(blocks)

    getMacros(blocks,'enabled',args.silent)
    getStructures(blocks,'enabled',args.silent)
    getFunctions(blocks,'enabled',args.silent)
    
    if(args.out.split('.')[-1]=='rst'):
        print "Generating reStructured text output...\n"
        createRst(args.out)
        return
    elif(args.out.split('.')[-1]=='pdf'):
        createMarkdown()
    else :
        print "Invalid output format specified.Aborting..\n"
        return
    
    graphName = ''
    if(args.drvpath != ''):
        graphName = dirTree(args.drvpath,args.exclude)

    if(args.path is not None):
        modifyTemplate(args.md,args.out,graphName,args.path,args.debug)
    else :
        modifyTemplate(args.md,args.out,graphName,args.debug)

    if args.dump == True:
        import pickle
        with open ('macros.pickle','wb') as fw:
            pickle.dump(macros,fw)
        fw.close()
        with open ('functions.pickle','wb') as fw:
            pickle.dump(functions,fw)
        fw.close()
        with open ('structures.pickle','wb') as fw:
            pickle.dump(structures,fw)
        fw.close()
    
    return

def preprocessing(content,flags):
    content = re.sub('__.*__[)][)]',' ',content)
    content = content.replace("    "," ")
    content = content.replace("("," ( ")
    content = content.replace(")"," ) ")
    content = content.replace("{"," { ")
    content = content.replace(";"," ; ")

    #BEWARE,BUG: It is assumed in a lot of places that there are only single spaces.So, do not remove the line below
    #BUG: Due to this,it is possible that if your comments have pandoc-markdown style tables, your indentation will be gone . 
    content = re.sub(' +',' ',content)
    #Splitting file on the basis of comments
    blocks=re.split(re.escape('/**')+'|'+re.escape('/*')+'|'+re.escape('*/'),content)
    
    if (flags):
        for i in range(len(blocks)/2):
            blocks[2*i+1]=blocks[2*i+1].replace("\n *","\n")
    
    #Remove "//" style comments
    for i in range(len(blocks)/2+1):
        blockSplit = re.split(re.escape('//'),blocks[2*i])
        if( len(blockSplit) >1):
            blocks[2*i] = blockSplit[0]
            for j in range(len(blockSplit)-1):
                blocks[2*i] = blocks[2*i] + '\n'.join(blockSplit[j+1].split('\n')[1:])
        blocks[2*i] = blocks[2*i].strip()

    for i in range(len(blocks)/2+1):
        blocks[2*i] = blocks[2*i].replace('*','* ')
    return blocks

def processBlocks(blocks):
    processedBlocks = []
    for i in range(len(blocks)/2+1):
        processedBlock = Block()
        processedBlock.body = blocks[2*i]
        processedBlocks.append(processedBlock)

    processedBlocks[0].primaryComment = ''
    for i in range(len(blocks)/2):
        if(blocks[2*i+1].find('< ') == 0):
            if(processedBlocks[i].primaryComment != ''):
                processedBlocks[i].secondaryComment = blocks[2*i+1][1:]
            else :
                processedBlocks[i].primaryComment = blocks[2*i+1][1:]
        else :
            processedBlocks[i+1].primaryComment = blocks[2*i+1]
    for block in processedBlocks:
        block.lessThanStatus = len(re.findall(r'<',block.body)) - len(re.findall(r'>',block.body))
        block.lessThanDepth = min( len(re.findall(r'<',block.body)) , len(re.findall(r'>',block.body)) )
        block.curlyStatus = len(re.findall(r'{',block.body)) - len(re.findall(r'}',block.body))
        block.curlyDepth = min( len(re.findall(r'{',block.body)) , len(re.findall(r'}',block.body)) )        
        block.paranStatus = len(re.findall(re.escape('('),block.body)) - len(re.findall(re.escape(')'),block.body))
        block.paranDepth = min( len(re.findall(re.escape('('),block.body)) , len(re.findall(re.escape(')'),block.body)) )
        block.squareStatus = len(re.findall(re.escape('['),block.body)) - len(re.findall(re.escape(']'),block.body))
        block.squareDepth = min( len(re.findall(re.escape('['),block.body)) , len(re.findall(re.escape(']'),block.body)) )    
    return processedBlocks

def doxygenCleanup(blocks):
    for block in blocks:
        #BUG:some bug forces me to use this if condition
        if(block.primaryComment != None):    
            block.primaryComment = block.primaryComment.replace('\\brief ','')
        if(block.secondaryComment != None):
            block.secondaryComment = block.secondaryComment.replace('\\brief ','')
    return blocks

def getMacros(blocks,status = 'enabled',silent=False):
    if(status == 'disabled'):
        return macros
    candidates = []
    for i in range(len(blocks)):
        candidates = re.split('\n',blocks[i].body)
        for candidate in candidates:
            candidate = candidate.strip()
            if (( (candidate.find('#define ') >= 0) | (candidate.find('#DEFINE ') >= 0) ) & (len(candidate.split())>2)):
                macro = Macro()
                linesplit=candidate.split(' ')
                macro.name = linesplit[1].strip()
                macro.value = ''.join(linesplit[2:])

                macroComment=blocks[i].primaryComment
                macroCommentSplit = macroComment.split('\\anchor ')
                if(len(macroCommentSplit)==2):
                    macro.anchor = macroCommentSplit[1].split('\n')[0].strip()
                    #BUG:Check this line
                    macroComment = macroComment.replace('\\anchor '+macro.anchor,'')
                macroCommentSplit = macroComment.split('\\name ')
                if(len(macroCommentSplit)==2):
                    macro.doxygenName = macroCommentSplit[1].split('\n')[0]
                    #BUG:Check this line
                    macroComment = macroComment.replace('\\name '+macro.doxygenName,'')
                #BUG : this will put the LAST COMMENT
                macro.comment = macroComment
                if not silent:
                    print "Found macro: "+macro.name
                macros.append(macro)
    return

#BUG: Asssumes that the comments for variables are after declaration .CHECK!!!
def getStructures(blocks,status = 'enabled',silent=False):
    if(status == 'disabled'):
        return structures
    candidate = ''
    candidateSplit=[]
    for block in blocks:
        blockSplit = block.body.split('typedef struct')
        if(len(blockSplit)==2):
            if len(blockSplit[1].split('{')) > 0:
                candidate = block.body
                structure = Structure()
                structure.comment=block.primaryComment
                candidateSplit=blockSplit[1].split('{')[-1].split(';')
                for varCandidate in candidateSplit:
                    if(len(varCandidate.split())>=2):
                        variable= Variable()
                        variable.name = varCandidate.split()[-1].strip()
                        variable.type =  ' '.join(varCandidate.split()[0:-1])
                        structure.variables.append(variable)
                if(block.secondaryComment != ''):
                    structure.variables[-1].comment = block.secondaryComment
        elif (candidate != ''):
            if len(block.body.split('}'))>0:
                candidateSplit = block.body.split('}')[0].split(';')
            else :
                candidateSplit = block.body.split(';')

            for varCandidate in candidateSplit:
                if(len(varCandidate.split())>=2):
                    variable = Variable()
                    variable.name = varCandidate.split()[-1]
                    variable.type =  ' '.join(varCandidate.split()[0:-1])
                    structure.variables.append(variable)
            if(block.secondaryComment != ''):
                structure.variables[-1].comment = block.secondaryComment
            if len(block.body.split('}'))>=2:
                structure.newName = block.body.split('}')[1].split(';')[0].strip()
                if not silent:
                    print "Found C structure: "+structure.newName
                structures.append(structure)
                candidate = ''


    return

def getFunctions(blocks,status = 'enabled',silent=False):
    if(status == 'disabled'):
        return functions
    for i in range(len(blocks)):
        candidates = blocks[i].body.split(';')
        for candidate in candidates:
            candidate = candidate.strip()
            if( (candidate.find('{')<0) & (candidate.find('}')<0) & (candidate.find('#define')<0) & (candidate.find('#DEFINE')<0) ):
                funcSplit = re.split(re.escape('(')+'|'+re.escape(')'),candidate)
                if(len(funcSplit) == 3):
                    function = Function()
                    function.name = funcSplit[0].split()[-1].strip()
                    function.outputType = ' '.join(funcSplit[0].split()[0:-1])
                    function.comment = blocks[i].primaryComment
                    function.inputArgs = []
                    args = funcSplit[1].split(',')
                    for arg in args:
                        variable = Variable()
                        variable.name = arg.split()[-1].strip()
                        variable.type = ' '.join(arg.split()[0:-1])
                        #THINK: Should the variable name for the comment be considered only as the array_name and not as array_name[array_len]
                        formattedVariableName = variable.name.split('[')[0]
                        if(function.comment.find('\\param '+ formattedVariableName) > 0):
                            variable.comment = function.comment.split('\\param '+formattedVariableName)[1].split('\\param')[0].split('\\return')[0]
                            function.comment = function.comment.replace('\\param '+formattedVariableName+variable.comment,'')
                        function.inputArgs.append(variable)
                    #BUG:Using newline to separate returnValue descriptions;May be problematic.Find a better way to do this
                    if(function.comment.find('\\return ') > 0):
                        function.returnValue = function.comment.split('\\return ')[1].split('\n')[0]
                        function.comment = function.comment.replace('\\return '+function.returnValue,'')                    
                    function.comment = function.comment.strip()
                    
                    reqSplit = function.comment.split('DOX_REQ_TAG \\( ')
                    for i in range(len(reqSplit)-1):
                        function.requirements.append(reqSplit[i].split('DOX_REQ_TAG \\( ')[0].split()[0])

                    if (funcSplit[2].strip() != ''):
                        print "warning:"+function.name + " : this is not a function.\n"
                    else :
                        if not silent:
                            print "Found function: "+function.name
                        functions.append(function)
    return


def createRst(filename):
    #clear the file contents
    open(filename,'w').close()
    fw= open(filename,'a')
    #fw.write("\\newpage")
    fw.write("\nLow Level Definitions\n")
    fw.write("=========================\n\n")
    fw.write("Constants and Enumerations\n")
    fw.write("----------------------------\n\n")
    currAnchor = ''
    for macro in macros:
        if macro.anchor != '':
            if macro.anchor != currAnchor:
                if currAnchor != '':
                    fw.write("**Comments**\nNone\n\n")
                    fw.write("**Constraints**\nNone\n\n")
                    fw.write("**See Also**\nNone\n\n")
                currAnchor = macro.anchor
                fw.write(macro.anchor+"\n")
                fw.write('~'*(len(macro.anchor)+2)+'\n\n')
                fw.write(macro.comment+"  \n\n")
                fw.write("**Definitions**\n\n")
                fw.write(" #define "+macro.name+" "+macro.value+"\n\n")
            else :
                fw.write(" #define "+macro.name+" "+macro.value+"\n\n")

        else :
            currAnchor = ''
            fw.write(macro.name+"\n")
            fw.write('~'*(len(macro.name)+2)+'\n\n')
            fw.write(macro.comment+"  \n\n")
            fw.write("**Definition**\n\n #define "+macro.name+" "+macro.value+"\n\n")
            fw.write("**Comments**\nNone\n\n")
            fw.write("**Constraints**\nNone\n\n")
            fw.write("**See Also**\nNone\n\n")

    fw.write("Typedefs and Data Structures\n")
    fw.write("-------------------------------\n\n")
    for structure in structures:
        fw.write(structure.newName+"\n")
        fw.write('~'*(len(structure.newName)+2)+'\n\n')
        fw.write(structure.comment+"\n\n")
        
        fw.write("**Definition**\n\n")
        fw.write("typedef struct  \n{  \n")
        for variable in structure.variables:
            fw.write("    "+variable.type+" "+variable.name+";  \n")
        fw.write("} "+structure.newName+";  \n\n\n")

        fw.write("**Fields**\n\n")
        for variable in structure.variables:
            fw.write("* "+variable.name+" : "+variable.comment+"\n")
        fw.write("\n")

        fw.write("**Comments**\nNone\n\n")
        fw.write("**Constraints**\nNone\n\n")
        fw.write("**See Also**\nNone\n\n")
    
    fw.write("API Definition\n")
    fw.write("-------------------\n\n")
    for function in functions:
        fw.write(function.name+"\n")
        fw.write('~'*(len(function.name)+2)+'\n\n')
        #Space added at the start to stop weird udma.h problem.
        fw.write(" "+function.comment+"\n\n")
        
        fw.write("**Syntax**\n\n")
        fw.write(function.outputType+" "+function.name+"(")
        fw.write(function.inputArgs[0].type+" "+function.inputArgs[0].name)
        for i in range(len(function.inputArgs)-1):
            fw.write(","+function.inputArgs[i+1].type+" "+function.inputArgs[i+1].name)
        fw.write(");\n\n")

        fw.write("**Arguments**\n\n")
        for variable in function.inputArgs:
            fw.write(' '+variable.name+' : '+variable.comment+'  \n')
        fw.write('\n\n')

        fw.write("**Return Value**\n")
        if(function.returnValue.strip() == ''):
            fw.write('None\n\n')
        else :
            fw.write(function.returnValue+'\n\n')

        fw.write("**Comments**\nNone\n\n")
        fw.write("**Constraints**\nNone\n\n")
        fw.write("**See Also**\nNone\n\n")

    fw.close()
    return

def createMarkdown():
    #clear the file contents
    open(autoFilename,'w').close()
    fw= open(autoFilename,'a')
    #fw.write("\\newpage")
    fw.write("\n# Low Level Definitions\n\n")

    fw.write("## Constants and Enumerations\n")
    currAnchor = ''
    for macro in macros:
        if macro.anchor != '':
            if macro.anchor != currAnchor:
                if currAnchor != '':
                    fw.write("**Comments**\nNone\n\n")
                    fw.write("**Constraints**\nNone\n\n")
                    fw.write("**See Also**\nNone\n\n")
                currAnchor = macro.anchor
                fw.write("### "+macro.anchor+"\n")
                fw.write(macro.comment+"  \n\n")
                fw.write("**Definitions**\n\n")
                fw.write(" #define "+macro.name+" "+macro.value+"\n\n")
            else :
                fw.write(" #define "+macro.name+" "+macro.value+"\n\n")

        else :
            currAnchor = ''
            fw.write("### "+macro.name+"\n")
            fw.write(macro.comment+"  \n\n")
            fw.write("**Definition**\n\n #define "+macro.name+" "+macro.value+"\n\n")
            fw.write("**Comments**\nNone\n\n")
            fw.write("**Constraints**\nNone\n\n")
            fw.write("**See Also**\nNone\n\n")

    fw.write("## Typedefs and Data Structures\n")
    for structure in structures:
        fw.write("### "+structure.newName+"\n")
        fw.write(structure.comment+"\n\n")
        
        fw.write("**Definition**\n\n")
        fw.write("typedef struct  \n{  \n")
        for variable in structure.variables:
            fw.write("    "+variable.type+" "+variable.name+";  \n")
        fw.write("} "+structure.newName+";  \n\n\n")

        fw.write("**Fields**\n\n")
        for variable in structure.variables:
            fw.write("* "+variable.name+" : "+variable.comment+"\n")
        fw.write("\n")

        fw.write("**Comments**\nNone\n\n")
        fw.write("**Constraints**\nNone\n\n")
        fw.write("**See Also**\nNone\n\n")
    
    fw.write("## API Definition\n")
    for function in functions:
        fw.write("### "+function.name+"\n")
        #Space added at the start to stop weird udma.h problem.
        fw.write(" "+function.comment+"\n\n")
        
        fw.write("**Syntax**\n\n")
        fw.write(function.outputType+" "+function.name+"(")
        fw.write(function.inputArgs[0].type+" "+function.inputArgs[0].name)
        for i in range(len(function.inputArgs)-1):
            fw.write(","+function.inputArgs[i+1].type+" "+function.inputArgs[i+1].name)
        fw.write(");\n\n")

        fw.write("**Arguments**\n\n")
        for variable in function.inputArgs:
            fw.write(' '+variable.name+' : '+variable.comment+'  \n')
        fw.write('\n\n')

        fw.write("**Return Value**\n")
        if(function.returnValue.strip() == ''):
            fw.write('None\n\n')
        else :
            fw.write(function.returnValue+'\n\n')

        fw.write("**Comments**\nNone\n\n")
        fw.write("**Constraints**\nNone\n\n")
        fw.write("**See Also**\nNone\n\n")

    fw.close()

    return


def postProcessing(autoContent):
    
    #Limitation:These lines are added because some syntax like '\ref' are supported by doxygen,
    #   but pandoc has problems when backslash is used with unknown syntax.
    #BUG:Should ideally be replace('\\n\n','\n').Check why this is not working .
    autoContent=autoContent.replace('\\n','  ')
    autoContent=autoContent.replace('\\ref ','')
    autoContent=autoContent.replace('@ {','')
    autoContent=autoContent.replace('@ }','')
    if(autoContent.find('\\param ') > 0):
        paramName = autoContent.split('\\param ')[1].split()[0]
        print "warning: Unclear parameter definition:" + paramName + " found.Check the definition in the comment."
        autoContent=autoContent.replace('\\param ','Parameter: ')
    if(autoContent.find('\\name ') > 0):
        anchorName = autoContent.split('\\name ')[1].split()[0]
        print "warning: Anchor name:" + anchorName + " cannot be understood.Check the comment."
        autoContent=autoContent.replace('\\name ','Group name:')

    for macro in macros:
        if macro.anchor != '':
            autoContent =  autoContent.replace(macro.anchor+' ','['+macro.anchor+'] ')
            #autoContent =  re.sub(macro.anchor+r'\s','['+macro.anchor+'] ',autoContent)
        else:
            autoContent =  autoContent.replace(macro.name+' ','['+macro.name+'] ')
            #autoContent =  re.sub(macro.name+r'\s','['+macro.name+'] ',autoContent)
    for structure in structures:
        if structure.newName != '':
            autoContent =  autoContent.replace(structure.newName+' ','['+structure.newName+'] ')
            #autoContent =  re.sub(structure.newName+r'\s','['+structure.newName+'] ',autoContent)
        if structure.oldName != '':
            autoContent =  autoContent.replace(structure.oldName+' ','['+structure.oldName+'] ')
            #autoContent =  re.sub(structure.oldName+r'\s','['+structure.oldName+'] ',autoContent)
    for function in functions:
        if (function.name != None) & (function.name.strip() != ''):
            autoContent =  autoContent.replace(function.name+' ','['+function.name+'] ')
            #autoContent =  re.sub(function.name+r'\s','['+function.name+'] ',autoContent)

    return autoContent

def modifyTemplate(inputFilename,pdfName,graphName,graphicsPath='./',debug=False):
    #Handling the case when no template is given.
    fr= open(autoFilename,'r')
    lines = fr.readlines()
    autoContent = "".join(lines)    
    fr.close()
    if (inputFilename == '') | (inputFilename == None):
        autoContent = postProcessing(autoContent)
        open(autoFilename,'w').close()
        fw = open(autoFilename,'a')
        fw.write(autoContent)
        fw.close()
        
        print "pandoc -s -V geometry:margin=1in --number-sections --toc -o "+pdfName+' '+autoFilename
        call("pandoc -s -V geometry:margin=1in --number-sections --toc -o "+pdfName+' '+autoFilename,shell=True)        
        #remove temporary files
        if not debug:
            call("rm "+autoFilename,shell=True)
        return

    #Handling the case when a template is given.
    tempFilename="temp"+re.split(re.escape('/')+'|'+re.escape("\\"),inputFilename)[-1]
    #print "cp "+inputFilename+" "+tempFilename 
    call("cp "+inputFilename+" "+tempFilename,shell=True)

    fr= open(tempFilename,'r')
    lines = fr.readlines()
    manualContent = "".join(lines)    
    fr.close()

    autoContent = postProcessing(autoContent)
    #autoContent=re.sub(r'\\n\n','\n',autoContent)
    manualContent = manualContent.replace("GRAPHICS_ABSOLUTE_PATH",graphicsPath)
    manualContent = manualContent.replace("@low_level_definitions",autoContent)

    if graphName != '':
        manualContent = manualContent.replace('@directory_structure','![Directory structure]('+graphName+'.png)')
    #clear temporary .md file contents
    open(tempFilename,'w').close()
    fw = open(tempFilename,'a')
    fw.write(manualContent)
    fw.close()
    
    print "pandoc -s -V geometry:margin=1in --toc --number-sections -o "+pdfName+' '+tempFilename
    call("pandoc -s -V geometry:margin=1in --toc --number-sections -o "+pdfName+' '+tempFilename,shell=True)
    #remove temporary files
    if not debug:
        call("rm "+tempFilename,shell=True)
        call("rm "+autoFilename,shell=True)
    return

def dirTree(drvpath,ignoredDirs):
#Example: python docGen.py --md ../docs/Sciclient_designDoc_content.md --pdf scicclient_des.pdf --source ../sciclient.h ../soc/V0/sciclient_pm.h ../soc/V0/sciclient_fmwMsgParams.h --path ../docs/ -s --drvpath ../ -x docs tools firmware
    from graphviz import Digraph

    drvpath = os.path.realpath(drvpath)
    drvname = os.path.relpath(drvpath,drvpath+'/../')
    print "Generating directory structure for "+drvname+" ..." 

    graphName = drvname+'_dirTree'
    g = Digraph('G', filename=graphName,format='png')

    for root,subdirs,files in os.walk(drvpath):
        #Ignoring hidden files
        files = [f for f in files if not f[0] == '.']
        subdirs[:] = [d for d in subdirs if not d[0] == '.']

        modifiedRoot = root.replace(drvpath+'/','')   
        if modifiedRoot not in ignoredDirs:
            for subdir in subdirs:
                modifiedSubdir = os.path.relpath(os.path.join(root,subdir),drvpath)
                g.edge(modifiedRoot,modifiedSubdir)
                g.node(modifiedSubdir, shape='rect',style='filled')
            for file in files:
                g.edge(modifiedRoot, file)
        else :
            for subdir in subdirs:
                modifiedSubdir = os.path.relpath(os.path.join(root,subdir),drvpath)
                ignoredDirs.append(modifiedSubdir)
    g.view()
    return graphName

if __name__ == '__main__':
    pre_main()


#BUG: Comments before parameter definition in struct may be problematic. CHECK!!
#BUG: Fix the bug when comments for a macro in an anchor exists alongwith the comment for the anchor and document how to use anchors and their limitations.
#BUG: dirTree not working on windows
#BUG: If output is .rst ,returns ad soon as output is generated.
#TODO: Add documentation for plain "struct" definitions alongwith the existing "typedef struct" definitions
