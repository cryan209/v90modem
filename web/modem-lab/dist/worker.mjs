import {runPair} from './engine-pair.mjs';
self.onmessage=async({data})=>{
  try{const result=await runPair(data.mode,data.text,p=>self.postMessage({type:'progress',...p}),()=>false,data.startup);
    self.postMessage({type:'result',...result},[result.left.buffer,result.right.buffer]);
  }catch(error){self.postMessage({type:'error',message:error.message});}
};
