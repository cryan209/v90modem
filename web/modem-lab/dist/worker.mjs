import {runPair,runReliability} from './engine-pair.mjs';
self.onmessage=async({data})=>{
  try{
    if(data.task==='reliability'){
      const result=await runReliability(data.settings,p=>self.postMessage({type:'reliability-progress',...p}));
      self.postMessage({type:'reliability-result',...result});
    }else{
      const result=await runPair(data.mode,data.text,p=>self.postMessage({type:'progress',...p}),()=>false,data.startup);
      self.postMessage({type:'result',...result},[result.left.buffer,result.right.buffer]);
    }
  }catch(error){self.postMessage({type:'error',message:error.message});}
};
